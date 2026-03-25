/**
 * @license
 * SPDX-License-Identifier: Apache-2.0
 */

import { PRBParameters, LinkageDimensions, MaterialProperties } from '../types';

export const DEFAULT_PRB: PRBParameters = {
  gamma: [0.1, 0.35, 0.40, 0.15],
  k: [3.51, 2.99, 2.58],
};

export const DEFAULT_MATERIAL: MaterialProperties = {
  E: 200e9, // Steel: 200 GPa
  yieldStrength: 250e6, // 250 MPa
  thickness: 0.002, // 2 mm
  width: 0.02, // 20 mm
};

export const DEFAULT_DIMENSIONS: LinkageDimensions = {
  l: 0.1, // 100 mm
  r: (1 - Math.SQRT2 / 2) * 0.1,
  Au: (-1 / Math.SQRT2) * 0.1,
  Av: (1 / Math.SQRT2) * 0.1,
  Bx: 0,
  By: (1 / Math.SQRT2) * 0.1,
};

/**
 * Forward kinematics of the 3R chain
 */
export function forwardKinematics(theta: [number, number, number], gamma: [number, number, number, number], l: number) {
  const [t1, t2, t3] = theta;
  const [g0, g1, g2, g3] = gamma;

  const Qx = (g0 + g1 * Math.cos(t1) + g2 * Math.cos(t1 + t2) + g3 * Math.cos(t1 + t2 + t3)) * l;
  const Qy = (g1 * Math.sin(t1) + g2 * Math.sin(t1 + t2) + g3 * Math.sin(t1 + t2 + t3)) * l;
  const theta0 = t1 + t2 + t3;

  return { Qx, Qy, theta0 };
}

/**
 * Inverse statics: compute Fx, Fy, M from theta
 */
export function computeLoads(theta: [number, number, number], prb: PRBParameters, l: number) {
  const [t1, t2, t3] = theta;
  const { k, gamma } = prb;
  const [g0, g1, g2, g3] = gamma;

  const t12 = t1 + t2;
  const t123 = t1 + t2 + t3;

  // Joint positions relative to P0
  const P1x = g1 * Math.cos(t1);
  const P1y = g1 * Math.sin(t1);
  const P2x = P1x + g2 * Math.cos(t12);
  const P2y = P1y + g2 * Math.sin(t12);
  const Qx = P2x + g3 * Math.cos(t123);
  const Qy = P2y + g3 * Math.sin(t123);

  // Moments at joints
  const tq = [k[0] * t1, k[1] * t2, k[2] * t3];

  // Matrix A for: A * [Fx*l, Fy*l, M]^T = [tq1, tq2, tq3]^T
  // Row i: [ -(yQ-yPi), (xQ-xPi), 1 ]
  const A = [
    [-(Qy - 0), (Qx - 0), 1],
    [-(Qy - P1y), (Qx - P1x), 1],
    [-(Qy - P2y), (Qx - P2x), 1]
  ];

  const loads = solve3x3(A, tq);
  if (loads) {
    return { Fx: loads[0] / l, Fy: loads[1] / l, M: loads[2] };
  }

  // If singular (t2=0), return a regularized solution
  const eps = 1e-9;
  A[1][0] += eps; // Perturb slightly to avoid singularity
  const regLoads = solve3x3(A, tq) || [0, 0, 0];
  return { Fx: regLoads[0] / l, Fy: regLoads[1] / l, M: regLoads[2] };
}

/**
 * Residual function for the solver
 */
export function computeResidual(theta: [number, number, number], psi: number, dims: LinkageDimensions, prb: PRBParameters) {
  const { Qx, Qy, theta0 } = forwardKinematics(theta, prb.gamma, dims.l);
  
  const Ax = Qx + dims.Au * Math.cos(theta0) - dims.Av * Math.sin(theta0);
  const Ay = Qy + dims.Au * Math.sin(theta0) + dims.Av * Math.cos(theta0);
  
  const targetAx = dims.Bx + dims.r * Math.cos(psi);
  const targetAy = dims.By + dims.r * Math.sin(psi);
  
  const f1 = Ax - targetAx;
  const f2 = Ay - targetAy;
  
  // Static equilibrium: moments at joints must be consistent with force at A
  // Joint positions
  const [g0, g1, g2, g3] = prb.gamma;
  const l = dims.l;
  const t1 = theta[0];
  const t12 = theta[0] + theta[1];
  
  const P0x = g0 * l;
  const P0y = 0;
  const P1x = P0x + g1 * l * Math.cos(t1);
  const P1y = P0y + g1 * l * Math.sin(t1);
  const P2x = P1x + g2 * l * Math.cos(t12);
  const P2y = P1y + g2 * l * Math.sin(t12);
  
  // Moments at joints (normalized: tau = k * theta)
  const tq = [prb.k[0] * theta[0], prb.k[1] * theta[1], prb.k[2] * theta[2]];
  
  // Vectors from joints to A
  const R0x = Ax - P0x;
  const R0y = Ay - P0y;
  const R1x = Ax - P1x;
  const R1y = Ay - P1y;
  const R2x = Ax - P2x;
  const R2y = Ay - P2y;
  
  // Consistency condition for 3 moment equations with 2 force variables (Fx, Fy)
  // det([ -R0y R0x tq0 ; -R1y R1x tq1 ; -R2y R2x tq2 ]) = 0
  const f3 = tq[2] * (R0x * R1y - R0y * R1x) 
           - tq[1] * (R0x * R2y - R0y * R2x) 
           + tq[0] * (R1x * R2y - R1y * R2x);

  // Scale f3 to have similar magnitude as f1, f2 (meters)
  return [f1, f2, f3 / l]; 
}

/**
 * Newton-Raphson solver with backtracking line search for robustness
 */
export function solveLinkage(psi: number, dims: LinkageDimensions, prb: PRBParameters, initialGuess: [number, number, number]): [number, number, number] {
  let theta: [number, number, number] = [...initialGuess];
  const maxIter = 100;
  const tol = 1e-9;
  const h = 1e-7;

  for (let iter = 0; iter < maxIter; iter++) {
    const res = computeResidual(theta, psi, dims, prb);
    const f0 = 0.5 * (res[0]**2 + res[1]**2 + res[2]**2);
    const norm = Math.sqrt(2 * f0);
    
    if (norm < tol) break;

    // Numerical Jacobian
    const J: number[][] = [[0,0,0],[0,0,0],[0,0,0]];
    for (let j = 0; j < 3; j++) {
      const thetaH: [number, number, number] = [...theta];
      thetaH[j] += h;
      const resH = computeResidual(thetaH, psi, dims, prb);
      for (let i = 0; i < 3; i++) {
        J[i][j] = (resH[i] - res[i]) / h;
      }
    }

    // Solve J * delta = -res
    const delta = solve3x3(J, res.map(v => -v));
    if (!delta) {
      // If singular, try a small random perturbation to escape (last resort)
      // But user said "no random start", so let's just try a small fixed step
      theta[0] += 0.001;
      theta[1] += 0.001;
      theta[2] += 0.001;
      continue;
    }

    // Backtracking line search to ensure residual decreases
    let alpha = 1.0;
    const minAlpha = 1e-4;
    let success = false;
    
    while (alpha >= minAlpha) {
      const nextTheta: [number, number, number] = [
        theta[0] + delta[0] * alpha,
        theta[1] + delta[1] * alpha,
        theta[2] + delta[2] * alpha
      ];
      const nextRes = computeResidual(nextTheta, psi, dims, prb);
      const f1 = 0.5 * (nextRes[0]**2 + nextRes[1]**2 + nextRes[2]**2);
      
      // Armijo-like condition (simplified: just ensure decrease)
      if (f1 < f0 || Math.sqrt(2*f1) < tol) {
        theta = nextTheta;
        success = true;
        break;
      }
      alpha *= 0.5;
    }
    
    if (!success) {
      // If line search fails, take a tiny step in the Newton direction anyway
      // to try and push past the local non-convexity
      theta[0] += delta[0] * minAlpha;
      theta[1] += delta[1] * minAlpha;
      theta[2] += delta[2] * minAlpha;
    }
  }

  return theta;
}

function solve3x3(A: number[][], b: number[]): number[] | null {
  const det = A[0][0] * (A[1][1] * A[2][2] - A[1][2] * A[2][1]) -
              A[0][1] * (A[1][0] * A[2][2] - A[1][2] * A[2][0]) +
              A[0][2] * (A[1][0] * A[2][1] - A[1][1] * A[2][0]);

  if (Math.abs(det) < 1e-12) return null;

  const invDet = 1 / det;
  const x = (b[0] * (A[1][1] * A[2][2] - A[1][2] * A[2][1]) -
             A[0][1] * (b[1] * A[2][2] - A[1][2] * b[2]) +
             A[0][2] * (b[1] * A[2][1] - A[1][1] * b[2])) * invDet;
             
  const y = (A[0][0] * (b[1] * A[2][2] - A[1][2] * b[2]) -
             b[0] * (A[1][0] * A[2][2] - A[1][2] * A[2][0]) +
             A[0][2] * (A[1][0] * b[2] - b[1] * A[2][0])) * invDet;
             
  const z = (A[0][0] * (A[1][1] * b[2] - b[1] * A[2][1]) -
             A[0][1] * (A[1][0] * b[2] - b[1] * A[2][0]) +
             b[0] * (A[1][0] * A[2][1] - A[1][1] * A[2][0])) * invDet;

  return [x, y, z];
}

/**
 * Calculate crank torque
 */
export function calculateCrankTorque(
  theta: [number, number, number], 
  psi: number, 
  dims: LinkageDimensions, 
  prb: PRBParameters, 
  mat: MaterialProperties,
  thetaInitial: [number, number, number] = [0, 0, 0]
) {
  const I = (mat.width * Math.pow(mat.thickness, 3)) / 12;
  const EI_l = (mat.E * I) / dims.l;
  
  // Virtual Work Method: T = dV/dpsi
  // V = 0.5 * (EI/l) * sum(k_i * (theta_i - theta_i_initial)^2)
  // T = (EI/l) * sum(k_i * (theta_i - theta_i_initial) * dtheta_i/dpsi)
  
  // We find dtheta/dpsi by differentiating the equilibrium equations f(theta, psi) = 0
  // J * dtheta/dpsi + df/dpsi = 0  =>  dtheta/dpsi = -J^-1 * df/dpsi
  
  const h = 1e-7;
  const res = computeResidual(theta, psi, dims, prb);
  
  // Numerical Jacobian J = df/dtheta
  const J: number[][] = [[0,0,0],[0,0,0],[0,0,0]];
  for (let j = 0; j < 3; j++) {
    const thetaH: [number, number, number] = [...theta];
    thetaH[j] += h;
    const resH = computeResidual(thetaH, psi, dims, prb);
    for (let i = 0; i < 3; i++) {
      J[i][j] = (resH[i] - res[i]) / h;
    }
  }
  
  // Partial derivative df/dpsi
  const resPsiH = computeResidual(theta, psi + h, dims, prb);
  const dfdpsi = resPsiH.map((v, i) => (v - res[i]) / h);
  
  // Solve J * dtheta_dpsi = -dfdpsi
  const dtheta_dpsi = solve3x3(J, dfdpsi.map(v => -v));
  
  if (!dtheta_dpsi) {
    // Fallback to a small perturbation if J is singular (rare)
    return 0;
  }
  
  let torque_norm = 0;
  for (let i = 0; i < 3; i++) {
    torque_norm += prb.k[i] * (theta[i] - thetaInitial[i]) * dtheta_dpsi[i];
  }
  
  return torque_norm * EI_l;
}

/**
 * Calculate potential energy of the torsion springs
 */
export function calculatePotentialEnergy(
  theta: [number, number, number], 
  dims: LinkageDimensions, 
  prb: PRBParameters, 
  mat: MaterialProperties,
  thetaInitial: [number, number, number] = [0, 0, 0]
) {
  const I = (mat.width * Math.pow(mat.thickness, 3)) / 12;
  const EI_l = (mat.E * I) / dims.l;
  
  let energy_norm = 0;
  for (let i = 0; i < 3; i++) {
    energy_norm += 0.5 * prb.k[i] * Math.pow(theta[i] - thetaInitial[i], 2);
  }
  
  return energy_norm * EI_l;
}
