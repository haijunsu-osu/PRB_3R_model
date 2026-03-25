/**
 * @license
 * SPDX-License-Identifier: Apache-2.0
 */

export interface PRBParameters {
  gamma: [number, number, number, number];
  k: [number, number, number];
}

export interface MaterialProperties {
  E: number; // Young's Modulus (Pa)
  yieldStrength: number; // Pa
  thickness: number; // m
  width: number; // m
}

export interface LinkageDimensions {
  l: number; // Beam length
  r: number; // Crank length
  Au: number; // Coupler attachment u
  Av: number; // Coupler attachment v
  Bx: number; // Ground joint x
  By: number; // Ground joint y
}

export interface SimulationState {
  theta: [number, number, number]; // PRB angles
  psi: number; // Crank angle
  torque: number; // Crank torque
}
