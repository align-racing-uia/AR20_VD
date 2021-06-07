/* Simscape target specific file.
 * This file is generated for the Simscape network associated with the solver block 'dempsystest_bjorn/Solver Configuration'.
 */

#include <math.h>
#include <string.h>
#include "pm_std.h"
#include "sm_std.h"
#include "ne_std.h"
#include "ne_dae.h"
#include "sm_ssci_run_time_errors.h"
#include "sm_RuntimeDerivedValuesBundle.h"
#include "dempsystest_bjorn_59d2bdba_5_geometries.h"

PmfMessageId dempsystest_bjorn_59d2bdba_5_checkDynamics(const
  RuntimeDerivedValuesBundle *rtdv, const double *state, const double *input,
  const double *inputDot, const double *inputDdot, const double *discreteState,
  const int *modeVector, double *errorResult, NeuDiagnosticManager *neDiagMgr)
{
  const double *rtdvd = rtdv->mDoubles.mValues;
  const int *rtdvi = rtdv->mInts.mValues;
  double xx[34];
  (void) rtdvd;
  (void) rtdvi;
  (void) input;
  (void) inputDot;
  (void) inputDdot;
  (void) discreteState;
  (void) modeVector;
  (void) neDiagMgr;
  xx[0] = 1.0;
  xx[1] = 4.475014772496788e-3;
  xx[2] = 0.5;
  xx[3] = xx[2] * state[34];
  xx[4] = sin(xx[3]);
  xx[5] = 0.01763972361731614;
  xx[6] = cos(xx[3]);
  xx[3] = xx[1] * xx[4] - xx[5] * xx[6];
  xx[7] = 0.02000638461961504;
  xx[8] = 0.01021201250743607;
  xx[9] = xx[1] * xx[6] + xx[5] * xx[4];
  xx[10] = xx[3] * xx[7] - xx[8] * xx[9];
  xx[11] = 0.9691346498545245;
  xx[12] = 0.2458594005622715;
  xx[13] = xx[11] * xx[6] - xx[12] * xx[4];
  xx[14] = xx[11] * xx[4] + xx[12] * xx[6];
  xx[15] = xx[14];
  xx[16] = xx[3];
  xx[17] = xx[9];
  xx[4] = xx[14] * xx[7];
  xx[6] = xx[14] * xx[8];
  xx[18] = xx[10];
  xx[19] = - xx[4];
  xx[20] = xx[6];
  pm_math_Vector3_cross_ra(xx + 15, xx + 18, xx + 21);
  xx[18] = 2.0;
  xx[19] = 0.02673029948304496;
  xx[20] = 0.03945399617968662;
  xx[24] = xx[19] * xx[9] - xx[3] * xx[20];
  xx[3] = xx[14] * xx[20];
  xx[9] = xx[14] * xx[19];
  xx[25] = xx[24];
  xx[26] = xx[3];
  xx[27] = - xx[9];
  pm_math_Vector3_cross_ra(xx + 15, xx + 25, xx + 28);
  xx[14] = 1.819549182087155e-4;
  xx[15] = (xx[10] * xx[13] + xx[21]) * xx[18] - (xx[13] * xx[24] + xx[28]) *
    xx[18] + xx[14];
  xx[10] = 0.143507688009519;
  xx[16] = xx[18] * (xx[22] - xx[4] * xx[13]) - xx[18] * (xx[29] + xx[3] * xx[13])
    - xx[10];
  xx[3] = 0.05446369265536674;
  xx[4] = (xx[6] * xx[13] + xx[23]) * xx[18] - (xx[30] - xx[9] * xx[13]) * xx[18]
    + xx[3];
  xx[6] = sqrt(xx[15] * xx[15] + xx[16] * xx[16] + xx[4] * xx[4]);
  if (xx[6] == 0.0) {
    return sm_ssci_recordRunTimeError(
      "sm:compiler:messages:simulationErrors:ForceUndefined",
      "'dempsystest_bjorn/Spring and Damper Force2' force is undefined since base and follower origins are coincident.",
      neDiagMgr);
  }

  xx[6] = xx[0] / xx[6];
  xx[4] = xx[2] * state[36];
  xx[9] = sin(xx[4]);
  xx[13] = cos(xx[4]);
  xx[4] = xx[5] * xx[9] - xx[1] * xx[13];
  xx[15] = xx[5] * xx[13] + xx[1] * xx[9];
  xx[1] = xx[8] * xx[4] - xx[15] * xx[7];
  xx[5] = xx[11] * xx[13] + xx[12] * xx[9];
  xx[16] = xx[11] * xx[9] - xx[12] * xx[13];
  xx[11] = xx[16];
  xx[12] = - xx[15];
  xx[13] = xx[4];
  xx[9] = xx[16] * xx[7];
  xx[7] = xx[16] * xx[8];
  xx[21] = xx[1];
  xx[22] = - xx[9];
  xx[23] = - xx[7];
  pm_math_Vector3_cross_ra(xx + 11, xx + 21, xx + 24);
  xx[8] = xx[15] * xx[20] - xx[19] * xx[4];
  xx[4] = xx[16] * xx[20];
  xx[15] = xx[16] * xx[19];
  xx[19] = xx[8];
  xx[20] = xx[4];
  xx[21] = xx[15];
  pm_math_Vector3_cross_ra(xx + 11, xx + 19, xx + 27);
  xx[11] = (xx[1] * xx[5] + xx[24]) * xx[18] - (xx[5] * xx[8] + xx[27]) * xx[18]
    + xx[14];
  xx[1] = xx[18] * (xx[25] - xx[9] * xx[5]) - xx[18] * (xx[28] + xx[4] * xx[5])
    + xx[10];
  xx[4] = (xx[26] - xx[7] * xx[5]) * xx[18] - (xx[15] * xx[5] + xx[29]) * xx[18]
    + xx[3];
  xx[3] = sqrt(xx[11] * xx[11] + xx[1] * xx[1] + xx[4] * xx[4]);
  if (xx[3] == 0.0) {
    return sm_ssci_recordRunTimeError(
      "sm:compiler:messages:simulationErrors:ForceUndefined",
      "'dempsystest_bjorn/Spring and Damper Force3' force is undefined since base and follower origins are coincident.",
      neDiagMgr);
  }

  xx[3] = xx[0] / xx[3];
  xx[1] = 0.9996758317937826;
  xx[4] = xx[2] * state[96];
  xx[5] = cos(xx[4]);
  xx[7] = 0.01780577773926495;
  xx[8] = sin(xx[4]);
  xx[4] = xx[1] * xx[5] - xx[7] * xx[8];
  xx[9] = 0.02718332249205364;
  xx[10] = 3.240922178968028e-4;
  xx[11] = 0.01819561954822274;
  xx[12] = xx[10] * xx[5] + xx[11] * xx[8];
  xx[13] = xx[10] * xx[8] - xx[11] * xx[5];
  xx[14] = 8.594904568580759e-3;
  xx[15] = xx[9] * xx[12] - xx[13] * xx[14];
  xx[16] = xx[1] * xx[8] + xx[7] * xx[5];
  xx[19] = xx[16];
  xx[20] = xx[13];
  xx[21] = xx[12];
  xx[5] = xx[16] * xx[14];
  xx[8] = xx[16] * xx[9];
  xx[22] = xx[15];
  xx[23] = xx[5];
  xx[24] = - xx[8];
  pm_math_Vector3_cross_ra(xx + 19, xx + 22, xx + 25);
  xx[17] = 5.002590631222067e-3;
  xx[22] = 0.05733821843436025;
  xx[23] = xx[13] * xx[17] + xx[22] * xx[12];
  xx[12] = xx[16] * xx[17];
  xx[13] = xx[16] * xx[22];
  xx[28] = - xx[23];
  xx[29] = xx[12];
  xx[30] = xx[13];
  pm_math_Vector3_cross_ra(xx + 19, xx + 28, xx + 31);
  xx[16] = 6.441204104588483e-3;
  xx[19] = (xx[4] * xx[15] + xx[25]) * xx[18] - (xx[31] - xx[4] * xx[23]) * xx
    [18] + xx[16];
  xx[15] = 0.07552154092641389;
  xx[20] = xx[18] * (xx[26] + xx[5] * xx[4]) - xx[18] * (xx[32] + xx[12] * xx[4])
    - xx[15];
  xx[5] = 0.1804750742326544;
  xx[12] = (xx[27] - xx[8] * xx[4]) * xx[18] - (xx[13] * xx[4] + xx[33]) * xx[18]
    - xx[5];
  xx[4] = sqrt(xx[19] * xx[19] + xx[20] * xx[20] + xx[12] * xx[12]);
  if (xx[4] == 0.0) {
    return sm_ssci_recordRunTimeError(
      "sm:compiler:messages:simulationErrors:ForceUndefined",
      "'dempsystest_bjorn/Subsystem/Spring and Damper Force' force is undefined since base and follower origins are coincident.",
      neDiagMgr);
  }

  xx[4] = xx[0] / xx[4];
  xx[8] = xx[2] * state[94];
  xx[2] = cos(xx[8]);
  xx[12] = sin(xx[8]);
  xx[8] = xx[1] * xx[2] + xx[7] * xx[12];
  xx[13] = xx[11] * xx[2] + xx[10] * xx[12];
  xx[19] = xx[11] * xx[12] - xx[10] * xx[2];
  xx[10] = xx[13] * xx[14] - xx[9] * xx[19];
  xx[11] = xx[1] * xx[12] - xx[7] * xx[2];
  xx[23] = xx[11];
  xx[24] = - xx[13];
  xx[25] = xx[19];
  xx[1] = xx[11] * xx[14];
  xx[2] = xx[11] * xx[9];
  xx[26] = xx[10];
  xx[27] = xx[1];
  xx[28] = xx[2];
  pm_math_Vector3_cross_ra(xx + 23, xx + 26, xx + 29);
  xx[7] = xx[13] * xx[17] + xx[22] * xx[19];
  xx[9] = xx[11] * xx[17];
  xx[12] = xx[11] * xx[22];
  xx[19] = xx[7];
  xx[20] = xx[9];
  xx[21] = - xx[12];
  pm_math_Vector3_cross_ra(xx + 23, xx + 19, xx + 26);
  xx[11] = (xx[8] * xx[10] + xx[29]) * xx[18] - (xx[8] * xx[7] + xx[26]) * xx[18]
    + xx[16];
  xx[7] = xx[18] * (xx[30] + xx[1] * xx[8]) - xx[18] * (xx[27] + xx[9] * xx[8])
    + xx[15];
  xx[1] = (xx[2] * xx[8] + xx[31]) * xx[18] - (xx[28] - xx[12] * xx[8]) * xx[18]
    - xx[5];
  xx[2] = sqrt(xx[11] * xx[11] + xx[7] * xx[7] + xx[1] * xx[1]);
  if (xx[2] == 0.0) {
    return sm_ssci_recordRunTimeError(
      "sm:compiler:messages:simulationErrors:ForceUndefined",
      "'dempsystest_bjorn/Subsystem/Spring and Damper Force1' force is undefined since base and follower origins are coincident.",
      neDiagMgr);
  }

  xx[2] = xx[0] / xx[2];
  errorResult[0] = xx[6] + xx[3] + xx[4] + xx[2];
  return NULL;
}
