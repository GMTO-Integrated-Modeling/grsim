# CLOSED LOOP CALIBRATION 

Computes the matrices to derive the AGWS closed-loop calibration matrices
 * $D_{2oa}$: on-axis calibration of M2 RBM
 * $D_{2agws}$: off-axis calibration of M2 modes
 * $D_{1oa}$: on-axis calibration of M1 RBM
 * $D_{1agws}$: off-axis calibration of M1 RBM

The calibration algoritm is given by:
 * ASM/AGWS interaction: $s_{agws} = D_{2agws} b$ , where b are M2 modes
 * ASM/on-axis SH interaction: $s_{sh} = D_{2oa} b$
 * M1/AGWS interaction: $s_{agws} = D_{1agws} a$ , where a are the M1 modes
 * M1/on-axis SH interaction: $s_{sh} = D_{1oa} a$
 * M2 command to compensate M1 modes:
   * $D_{1oa} a = D_{2oa} b$ => $b = D_{2oa}^{-1} D_{1oa} a$
 * residual AGWS slopes after ASM correction of M1 modes with M2 modes:
   * $\varepsilon_{agws} = D_{1gws} a - D_{2gws} D_{2oa}^{-1} D_{1oa} a$