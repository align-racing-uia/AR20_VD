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
#include "sm_CTarget.h"

void dempsystest_bjorn_59d2bdba_5_setTargets(const RuntimeDerivedValuesBundle
  *rtdv, CTarget *targets)
{
  (void) rtdv;
  (void) targets;
}

void dempsystest_bjorn_59d2bdba_5_resetAsmStateVector(const void *mech, double
  *state)
{
  double xx[2];
  (void) mech;
  xx[0] = 0.0;
  xx[1] = 1.0;
  state[0] = xx[0];
  state[1] = xx[0];
  state[2] = xx[0];
  state[3] = xx[1];
  state[4] = xx[0];
  state[5] = xx[0];
  state[6] = xx[0];
  state[7] = xx[0];
  state[8] = xx[0];
  state[9] = xx[0];
  state[10] = xx[0];
  state[11] = xx[0];
  state[12] = xx[0];
  state[13] = xx[1];
  state[14] = xx[0];
  state[15] = xx[0];
  state[16] = xx[0];
  state[17] = xx[0];
  state[18] = xx[0];
  state[19] = xx[0];
  state[20] = xx[1];
  state[21] = xx[0];
  state[22] = xx[0];
  state[23] = xx[0];
  state[24] = xx[0];
  state[25] = xx[0];
  state[26] = xx[0];
  state[27] = xx[1];
  state[28] = xx[0];
  state[29] = xx[0];
  state[30] = xx[0];
  state[31] = xx[0];
  state[32] = xx[0];
  state[33] = xx[0];
  state[34] = xx[0];
  state[35] = xx[0];
  state[36] = xx[0];
  state[37] = xx[0];
  state[38] = xx[1];
  state[39] = xx[0];
  state[40] = xx[0];
  state[41] = xx[0];
  state[42] = xx[0];
  state[43] = xx[0];
  state[44] = xx[0];
  state[45] = xx[1];
  state[46] = xx[0];
  state[47] = xx[0];
  state[48] = xx[0];
  state[49] = xx[0];
  state[50] = xx[0];
  state[51] = xx[0];
  state[52] = xx[1];
  state[53] = xx[0];
  state[54] = xx[0];
  state[55] = xx[0];
  state[56] = xx[0];
  state[57] = xx[0];
  state[58] = xx[0];
  state[59] = xx[1];
  state[60] = xx[0];
  state[61] = xx[0];
  state[62] = xx[0];
  state[63] = xx[0];
  state[64] = xx[0];
  state[65] = xx[0];
  state[66] = xx[1];
  state[67] = xx[0];
  state[68] = xx[0];
  state[69] = xx[0];
  state[70] = xx[0];
  state[71] = xx[0];
  state[72] = xx[0];
  state[73] = xx[1];
  state[74] = xx[0];
  state[75] = xx[0];
  state[76] = xx[0];
  state[77] = xx[0];
  state[78] = xx[0];
  state[79] = xx[0];
  state[80] = xx[1];
  state[81] = xx[0];
  state[82] = xx[0];
  state[83] = xx[0];
  state[84] = xx[0];
  state[85] = xx[0];
  state[86] = xx[0];
  state[87] = xx[1];
  state[88] = xx[0];
  state[89] = xx[0];
  state[90] = xx[0];
  state[91] = xx[0];
  state[92] = xx[0];
  state[93] = xx[0];
  state[94] = xx[0];
  state[95] = xx[0];
  state[96] = xx[0];
  state[97] = xx[0];
  state[98] = xx[1];
  state[99] = xx[0];
  state[100] = xx[0];
  state[101] = xx[0];
  state[102] = xx[0];
  state[103] = xx[0];
  state[104] = xx[0];
  state[105] = xx[1];
  state[106] = xx[0];
  state[107] = xx[0];
  state[108] = xx[0];
  state[109] = xx[0];
  state[110] = xx[0];
  state[111] = xx[0];
  state[112] = xx[1];
  state[113] = xx[0];
  state[114] = xx[0];
  state[115] = xx[0];
  state[116] = xx[0];
  state[117] = xx[0];
  state[118] = xx[0];
  state[119] = xx[1];
  state[120] = xx[0];
  state[121] = xx[0];
  state[122] = xx[0];
  state[123] = xx[0];
  state[124] = xx[0];
  state[125] = xx[0];
  state[126] = xx[1];
  state[127] = xx[0];
  state[128] = xx[0];
  state[129] = xx[0];
  state[130] = xx[0];
  state[131] = xx[0];
  state[132] = xx[0];
}

void dempsystest_bjorn_59d2bdba_5_initializeTrackedAngleState(const void *mech,
  const RuntimeDerivedValuesBundle *rtdv, const int *modeVector, const double
  *motionData, double *state)
{
  const double *rtdvd = rtdv->mDoubles.mValues;
  const int *rtdvi = rtdv->mInts.mValues;
  (void) mech;
  (void) rtdvd;
  (void) rtdvi;
  (void) state;
  (void) modeVector;
  (void) motionData;
}

void dempsystest_bjorn_59d2bdba_5_computeDiscreteState(const void *mech, const
  RuntimeDerivedValuesBundle *rtdv, double *state)
{
  const double *rtdvd = rtdv->mDoubles.mValues;
  const int *rtdvi = rtdv->mInts.mValues;
  (void) mech;
  (void) rtdvd;
  (void) rtdvi;
  (void) state;
}

void dempsystest_bjorn_59d2bdba_5_adjustPosition(const void *mech, const double *
  dofDeltas, double *state)
{
  double xx[92];
  (void) mech;
  xx[0] = state[3];
  xx[1] = state[4];
  xx[2] = state[5];
  xx[3] = state[6];
  xx[4] = dofDeltas[3];
  xx[5] = dofDeltas[4];
  xx[6] = dofDeltas[5];
  pm_math_Quaternion_compDeriv_ra(xx + 0, xx + 4, xx + 7);
  xx[0] = state[3] + xx[7];
  xx[1] = state[4] + xx[8];
  xx[2] = state[5] + xx[9];
  xx[3] = state[6] + xx[10];
  xx[4] = 1.0e-64;
  xx[5] = sqrt(xx[0] * xx[0] + xx[1] * xx[1] + xx[2] * xx[2] + xx[3] * xx[3]);
  if (xx[4] > xx[5])
    xx[5] = xx[4];
  xx[6] = state[13];
  xx[7] = state[14];
  xx[8] = state[15];
  xx[9] = state[16];
  xx[10] = dofDeltas[6];
  xx[11] = dofDeltas[7];
  xx[12] = dofDeltas[8];
  pm_math_Quaternion_compDeriv_ra(xx + 6, xx + 10, xx + 13);
  xx[6] = state[13] + xx[13];
  xx[7] = state[14] + xx[14];
  xx[8] = state[15] + xx[15];
  xx[9] = state[16] + xx[16];
  xx[10] = sqrt(xx[6] * xx[6] + xx[7] * xx[7] + xx[8] * xx[8] + xx[9] * xx[9]);
  if (xx[4] > xx[10])
    xx[10] = xx[4];
  xx[11] = state[20];
  xx[12] = state[21];
  xx[13] = state[22];
  xx[14] = state[23];
  xx[15] = dofDeltas[9];
  xx[16] = dofDeltas[10];
  xx[17] = dofDeltas[11];
  pm_math_Quaternion_compDeriv_ra(xx + 11, xx + 15, xx + 18);
  xx[11] = state[20] + xx[18];
  xx[12] = state[21] + xx[19];
  xx[13] = state[22] + xx[20];
  xx[14] = state[23] + xx[21];
  xx[15] = sqrt(xx[11] * xx[11] + xx[12] * xx[12] + xx[13] * xx[13] + xx[14] *
                xx[14]);
  if (xx[4] > xx[15])
    xx[15] = xx[4];
  xx[16] = state[27];
  xx[17] = state[28];
  xx[18] = state[29];
  xx[19] = state[30];
  xx[20] = dofDeltas[12];
  xx[21] = dofDeltas[13];
  xx[22] = dofDeltas[14];
  pm_math_Quaternion_compDeriv_ra(xx + 16, xx + 20, xx + 23);
  xx[16] = state[27] + xx[23];
  xx[17] = state[28] + xx[24];
  xx[18] = state[29] + xx[25];
  xx[19] = state[30] + xx[26];
  xx[20] = sqrt(xx[16] * xx[16] + xx[17] * xx[17] + xx[18] * xx[18] + xx[19] *
                xx[19]);
  if (xx[4] > xx[20])
    xx[20] = xx[4];
  xx[21] = state[38];
  xx[22] = state[39];
  xx[23] = state[40];
  xx[24] = state[41];
  xx[25] = dofDeltas[17];
  xx[26] = dofDeltas[18];
  xx[27] = dofDeltas[19];
  pm_math_Quaternion_compDeriv_ra(xx + 21, xx + 25, xx + 28);
  xx[21] = state[38] + xx[28];
  xx[22] = state[39] + xx[29];
  xx[23] = state[40] + xx[30];
  xx[24] = state[41] + xx[31];
  xx[25] = sqrt(xx[21] * xx[21] + xx[22] * xx[22] + xx[23] * xx[23] + xx[24] *
                xx[24]);
  if (xx[4] > xx[25])
    xx[25] = xx[4];
  xx[26] = state[45];
  xx[27] = state[46];
  xx[28] = state[47];
  xx[29] = state[48];
  xx[30] = dofDeltas[20];
  xx[31] = dofDeltas[21];
  xx[32] = dofDeltas[22];
  pm_math_Quaternion_compDeriv_ra(xx + 26, xx + 30, xx + 33);
  xx[26] = state[45] + xx[33];
  xx[27] = state[46] + xx[34];
  xx[28] = state[47] + xx[35];
  xx[29] = state[48] + xx[36];
  xx[30] = sqrt(xx[26] * xx[26] + xx[27] * xx[27] + xx[28] * xx[28] + xx[29] *
                xx[29]);
  if (xx[4] > xx[30])
    xx[30] = xx[4];
  xx[31] = state[52];
  xx[32] = state[53];
  xx[33] = state[54];
  xx[34] = state[55];
  xx[35] = dofDeltas[23];
  xx[36] = dofDeltas[24];
  xx[37] = dofDeltas[25];
  pm_math_Quaternion_compDeriv_ra(xx + 31, xx + 35, xx + 38);
  xx[31] = state[52] + xx[38];
  xx[32] = state[53] + xx[39];
  xx[33] = state[54] + xx[40];
  xx[34] = state[55] + xx[41];
  xx[35] = sqrt(xx[31] * xx[31] + xx[32] * xx[32] + xx[33] * xx[33] + xx[34] *
                xx[34]);
  if (xx[4] > xx[35])
    xx[35] = xx[4];
  xx[36] = state[59];
  xx[37] = state[60];
  xx[38] = state[61];
  xx[39] = state[62];
  xx[40] = dofDeltas[26];
  xx[41] = dofDeltas[27];
  xx[42] = dofDeltas[28];
  pm_math_Quaternion_compDeriv_ra(xx + 36, xx + 40, xx + 43);
  xx[36] = state[59] + xx[43];
  xx[37] = state[60] + xx[44];
  xx[38] = state[61] + xx[45];
  xx[39] = state[62] + xx[46];
  xx[40] = sqrt(xx[36] * xx[36] + xx[37] * xx[37] + xx[38] * xx[38] + xx[39] *
                xx[39]);
  if (xx[4] > xx[40])
    xx[40] = xx[4];
  xx[41] = state[66];
  xx[42] = state[67];
  xx[43] = state[68];
  xx[44] = state[69];
  xx[45] = dofDeltas[29];
  xx[46] = dofDeltas[30];
  xx[47] = dofDeltas[31];
  pm_math_Quaternion_compDeriv_ra(xx + 41, xx + 45, xx + 48);
  xx[41] = state[66] + xx[48];
  xx[42] = state[67] + xx[49];
  xx[43] = state[68] + xx[50];
  xx[44] = state[69] + xx[51];
  xx[45] = sqrt(xx[41] * xx[41] + xx[42] * xx[42] + xx[43] * xx[43] + xx[44] *
                xx[44]);
  if (xx[4] > xx[45])
    xx[45] = xx[4];
  xx[46] = state[73];
  xx[47] = state[74];
  xx[48] = state[75];
  xx[49] = state[76];
  xx[50] = dofDeltas[32];
  xx[51] = dofDeltas[33];
  xx[52] = dofDeltas[34];
  pm_math_Quaternion_compDeriv_ra(xx + 46, xx + 50, xx + 53);
  xx[46] = state[73] + xx[53];
  xx[47] = state[74] + xx[54];
  xx[48] = state[75] + xx[55];
  xx[49] = state[76] + xx[56];
  xx[50] = sqrt(xx[46] * xx[46] + xx[47] * xx[47] + xx[48] * xx[48] + xx[49] *
                xx[49]);
  if (xx[4] > xx[50])
    xx[50] = xx[4];
  xx[51] = state[80];
  xx[52] = state[81];
  xx[53] = state[82];
  xx[54] = state[83];
  xx[55] = dofDeltas[35];
  xx[56] = dofDeltas[36];
  xx[57] = dofDeltas[37];
  pm_math_Quaternion_compDeriv_ra(xx + 51, xx + 55, xx + 58);
  xx[51] = state[80] + xx[58];
  xx[52] = state[81] + xx[59];
  xx[53] = state[82] + xx[60];
  xx[54] = state[83] + xx[61];
  xx[55] = sqrt(xx[51] * xx[51] + xx[52] * xx[52] + xx[53] * xx[53] + xx[54] *
                xx[54]);
  if (xx[4] > xx[55])
    xx[55] = xx[4];
  xx[56] = state[87];
  xx[57] = state[88];
  xx[58] = state[89];
  xx[59] = state[90];
  xx[60] = dofDeltas[38];
  xx[61] = dofDeltas[39];
  xx[62] = dofDeltas[40];
  pm_math_Quaternion_compDeriv_ra(xx + 56, xx + 60, xx + 63);
  xx[56] = state[87] + xx[63];
  xx[57] = state[88] + xx[64];
  xx[58] = state[89] + xx[65];
  xx[59] = state[90] + xx[66];
  xx[60] = sqrt(xx[56] * xx[56] + xx[57] * xx[57] + xx[58] * xx[58] + xx[59] *
                xx[59]);
  if (xx[4] > xx[60])
    xx[60] = xx[4];
  xx[61] = state[98];
  xx[62] = state[99];
  xx[63] = state[100];
  xx[64] = state[101];
  xx[65] = dofDeltas[43];
  xx[66] = dofDeltas[44];
  xx[67] = dofDeltas[45];
  pm_math_Quaternion_compDeriv_ra(xx + 61, xx + 65, xx + 68);
  xx[61] = state[98] + xx[68];
  xx[62] = state[99] + xx[69];
  xx[63] = state[100] + xx[70];
  xx[64] = state[101] + xx[71];
  xx[65] = sqrt(xx[61] * xx[61] + xx[62] * xx[62] + xx[63] * xx[63] + xx[64] *
                xx[64]);
  if (xx[4] > xx[65])
    xx[65] = xx[4];
  xx[66] = state[105];
  xx[67] = state[106];
  xx[68] = state[107];
  xx[69] = state[108];
  xx[70] = dofDeltas[46];
  xx[71] = dofDeltas[47];
  xx[72] = dofDeltas[48];
  pm_math_Quaternion_compDeriv_ra(xx + 66, xx + 70, xx + 73);
  xx[66] = state[105] + xx[73];
  xx[67] = state[106] + xx[74];
  xx[68] = state[107] + xx[75];
  xx[69] = state[108] + xx[76];
  xx[70] = sqrt(xx[66] * xx[66] + xx[67] * xx[67] + xx[68] * xx[68] + xx[69] *
                xx[69]);
  if (xx[4] > xx[70])
    xx[70] = xx[4];
  xx[71] = state[112];
  xx[72] = state[113];
  xx[73] = state[114];
  xx[74] = state[115];
  xx[75] = dofDeltas[49];
  xx[76] = dofDeltas[50];
  xx[77] = dofDeltas[51];
  pm_math_Quaternion_compDeriv_ra(xx + 71, xx + 75, xx + 78);
  xx[71] = state[112] + xx[78];
  xx[72] = state[113] + xx[79];
  xx[73] = state[114] + xx[80];
  xx[74] = state[115] + xx[81];
  xx[75] = sqrt(xx[71] * xx[71] + xx[72] * xx[72] + xx[73] * xx[73] + xx[74] *
                xx[74]);
  if (xx[4] > xx[75])
    xx[75] = xx[4];
  xx[76] = state[119];
  xx[77] = state[120];
  xx[78] = state[121];
  xx[79] = state[122];
  xx[80] = dofDeltas[52];
  xx[81] = dofDeltas[53];
  xx[82] = dofDeltas[54];
  pm_math_Quaternion_compDeriv_ra(xx + 76, xx + 80, xx + 83);
  xx[76] = state[119] + xx[83];
  xx[77] = state[120] + xx[84];
  xx[78] = state[121] + xx[85];
  xx[79] = state[122] + xx[86];
  xx[80] = sqrt(xx[76] * xx[76] + xx[77] * xx[77] + xx[78] * xx[78] + xx[79] *
                xx[79]);
  if (xx[4] > xx[80])
    xx[80] = xx[4];
  xx[81] = state[126];
  xx[82] = state[127];
  xx[83] = state[128];
  xx[84] = state[129];
  xx[85] = dofDeltas[55];
  xx[86] = dofDeltas[56];
  xx[87] = dofDeltas[57];
  pm_math_Quaternion_compDeriv_ra(xx + 81, xx + 85, xx + 88);
  xx[81] = state[126] + xx[88];
  xx[82] = state[127] + xx[89];
  xx[83] = state[128] + xx[90];
  xx[84] = state[129] + xx[91];
  xx[85] = sqrt(xx[81] * xx[81] + xx[82] * xx[82] + xx[83] * xx[83] + xx[84] *
                xx[84]);
  if (xx[4] > xx[85])
    xx[85] = xx[4];
  state[0] = state[0] + dofDeltas[0];
  state[1] = state[1] + dofDeltas[1];
  state[2] = state[2] + dofDeltas[2];
  state[3] = xx[0] / xx[5];
  state[4] = xx[1] / xx[5];
  state[5] = xx[2] / xx[5];
  state[6] = xx[3] / xx[5];
  state[13] = xx[6] / xx[10];
  state[14] = xx[7] / xx[10];
  state[15] = xx[8] / xx[10];
  state[16] = xx[9] / xx[10];
  state[20] = xx[11] / xx[15];
  state[21] = xx[12] / xx[15];
  state[22] = xx[13] / xx[15];
  state[23] = xx[14] / xx[15];
  state[27] = xx[16] / xx[20];
  state[28] = xx[17] / xx[20];
  state[29] = xx[18] / xx[20];
  state[30] = xx[19] / xx[20];
  state[34] = state[34] + dofDeltas[15];
  state[36] = state[36] + dofDeltas[16];
  state[38] = xx[21] / xx[25];
  state[39] = xx[22] / xx[25];
  state[40] = xx[23] / xx[25];
  state[41] = xx[24] / xx[25];
  state[45] = xx[26] / xx[30];
  state[46] = xx[27] / xx[30];
  state[47] = xx[28] / xx[30];
  state[48] = xx[29] / xx[30];
  state[52] = xx[31] / xx[35];
  state[53] = xx[32] / xx[35];
  state[54] = xx[33] / xx[35];
  state[55] = xx[34] / xx[35];
  state[59] = xx[36] / xx[40];
  state[60] = xx[37] / xx[40];
  state[61] = xx[38] / xx[40];
  state[62] = xx[39] / xx[40];
  state[66] = xx[41] / xx[45];
  state[67] = xx[42] / xx[45];
  state[68] = xx[43] / xx[45];
  state[69] = xx[44] / xx[45];
  state[73] = xx[46] / xx[50];
  state[74] = xx[47] / xx[50];
  state[75] = xx[48] / xx[50];
  state[76] = xx[49] / xx[50];
  state[80] = xx[51] / xx[55];
  state[81] = xx[52] / xx[55];
  state[82] = xx[53] / xx[55];
  state[83] = xx[54] / xx[55];
  state[87] = xx[56] / xx[60];
  state[88] = xx[57] / xx[60];
  state[89] = xx[58] / xx[60];
  state[90] = xx[59] / xx[60];
  state[94] = state[94] + dofDeltas[41];
  state[96] = state[96] + dofDeltas[42];
  state[98] = xx[61] / xx[65];
  state[99] = xx[62] / xx[65];
  state[100] = xx[63] / xx[65];
  state[101] = xx[64] / xx[65];
  state[105] = xx[66] / xx[70];
  state[106] = xx[67] / xx[70];
  state[107] = xx[68] / xx[70];
  state[108] = xx[69] / xx[70];
  state[112] = xx[71] / xx[75];
  state[113] = xx[72] / xx[75];
  state[114] = xx[73] / xx[75];
  state[115] = xx[74] / xx[75];
  state[119] = xx[76] / xx[80];
  state[120] = xx[77] / xx[80];
  state[121] = xx[78] / xx[80];
  state[122] = xx[79] / xx[80];
  state[126] = xx[81] / xx[85];
  state[127] = xx[82] / xx[85];
  state[128] = xx[83] / xx[85];
  state[129] = xx[84] / xx[85];
}

static void perturbAsmJointPrimitiveState_0_0(double mag, double *state)
{
  state[0] = state[0] + mag;
}

static void perturbAsmJointPrimitiveState_0_0v(double mag, double *state)
{
  state[0] = state[0] + mag;
  state[7] = state[7] - 0.875 * mag;
}

static void perturbAsmJointPrimitiveState_0_1(double mag, double *state)
{
  state[1] = state[1] + mag;
}

static void perturbAsmJointPrimitiveState_0_1v(double mag, double *state)
{
  state[1] = state[1] + mag;
  state[8] = state[8] - 0.875 * mag;
}

static void perturbAsmJointPrimitiveState_0_2(double mag, double *state)
{
  state[2] = state[2] + mag;
}

static void perturbAsmJointPrimitiveState_0_2v(double mag, double *state)
{
  state[2] = state[2] + mag;
  state[9] = state[9] - 0.875 * mag;
}

static void perturbAsmJointPrimitiveState_0_3(double mag, double *state)
{
  double xx[15];
  xx[0] = 1.0;
  xx[1] = fabs(mag);
  xx[2] = xx[0] / (xx[1] - floor(xx[1]) + 1.0e-9);
  xx[1] = sin(xx[2]);
  xx[3] = 0.0;
  xx[4] = cos(xx[2]);
  xx[5] = sin(2.0 * xx[2]);
  xx[2] = 0.5 * mag;
  xx[6] = sqrt(xx[1] * xx[1] + xx[4] * xx[4] + xx[5] * xx[5]);
  xx[7] = xx[6] == 0.0 ? 0.0 : xx[1] / xx[6];
  xx[8] = sin(xx[2]);
  xx[9] = xx[6] == 0.0 ? 0.0 : xx[4] / xx[6];
  xx[10] = xx[6] == 0.0 ? 0.0 : xx[5] / xx[6];
  xx[11] = xx[1] == xx[3] && xx[4] == xx[3] && xx[5] == xx[3] ? xx[0] : cos(xx[2]);
  xx[12] = xx[7] * xx[8];
  xx[13] = xx[9] * xx[8];
  xx[14] = xx[10] * xx[8];
  xx[0] = state[3];
  xx[1] = state[4];
  xx[2] = state[5];
  xx[3] = state[6];
  pm_math_Quaternion_compose_ra(xx + 11, xx + 0, xx + 4);
  state[3] = xx[4];
  state[4] = xx[5];
  state[5] = xx[6];
  state[6] = xx[7];
}

static void perturbAsmJointPrimitiveState_0_3v(double mag, double *state)
{
  double xx[15];
  xx[0] = 1.0;
  xx[1] = fabs(mag);
  xx[2] = xx[0] / (xx[1] - floor(xx[1]) + 1.0e-9);
  xx[1] = sin(xx[2]);
  xx[3] = 0.0;
  xx[4] = cos(xx[2]);
  xx[5] = sin(2.0 * xx[2]);
  xx[2] = 0.5 * mag;
  xx[6] = sqrt(xx[1] * xx[1] + xx[4] * xx[4] + xx[5] * xx[5]);
  xx[7] = xx[6] == 0.0 ? 0.0 : xx[1] / xx[6];
  xx[8] = sin(xx[2]);
  xx[9] = xx[6] == 0.0 ? 0.0 : xx[4] / xx[6];
  xx[10] = xx[6] == 0.0 ? 0.0 : xx[5] / xx[6];
  xx[11] = xx[1] == xx[3] && xx[4] == xx[3] && xx[5] == xx[3] ? xx[0] : cos(xx[2]);
  xx[12] = xx[7] * xx[8];
  xx[13] = xx[9] * xx[8];
  xx[14] = xx[10] * xx[8];
  xx[3] = state[3];
  xx[4] = state[4];
  xx[5] = state[5];
  xx[6] = state[6];
  pm_math_Quaternion_compose_ra(xx + 11, xx + 3, xx + 7);
  state[3] = xx[7];
  state[4] = xx[8];
  state[5] = xx[9];
  state[6] = xx[10];
  state[10] = state[10] + 1.2 * mag;
  state[11] = state[11] - xx[2];
  state[12] = state[12] + 0.9 * mag;
}

static void perturbAsmJointPrimitiveState_1_0(double mag, double *state)
{
  double xx[15];
  xx[0] = 1.0;
  xx[1] = fabs(mag);
  xx[2] = xx[0] / (xx[1] - floor(xx[1]) + 1.0e-9);
  xx[1] = sin(xx[2]);
  xx[3] = 0.0;
  xx[4] = cos(xx[2]);
  xx[5] = sin(2.0 * xx[2]);
  xx[2] = 0.5 * mag;
  xx[6] = sqrt(xx[1] * xx[1] + xx[4] * xx[4] + xx[5] * xx[5]);
  xx[7] = xx[6] == 0.0 ? 0.0 : xx[1] / xx[6];
  xx[8] = sin(xx[2]);
  xx[9] = xx[6] == 0.0 ? 0.0 : xx[4] / xx[6];
  xx[10] = xx[6] == 0.0 ? 0.0 : xx[5] / xx[6];
  xx[11] = xx[1] == xx[3] && xx[4] == xx[3] && xx[5] == xx[3] ? xx[0] : cos(xx[2]);
  xx[12] = xx[7] * xx[8];
  xx[13] = xx[9] * xx[8];
  xx[14] = xx[10] * xx[8];
  xx[0] = state[13];
  xx[1] = state[14];
  xx[2] = state[15];
  xx[3] = state[16];
  pm_math_Quaternion_compose_ra(xx + 11, xx + 0, xx + 4);
  state[13] = xx[4];
  state[14] = xx[5];
  state[15] = xx[6];
  state[16] = xx[7];
}

static void perturbAsmJointPrimitiveState_1_0v(double mag, double *state)
{
  double xx[15];
  xx[0] = 1.0;
  xx[1] = fabs(mag);
  xx[2] = xx[0] / (xx[1] - floor(xx[1]) + 1.0e-9);
  xx[1] = sin(xx[2]);
  xx[3] = 0.0;
  xx[4] = cos(xx[2]);
  xx[5] = sin(2.0 * xx[2]);
  xx[2] = 0.5 * mag;
  xx[6] = sqrt(xx[1] * xx[1] + xx[4] * xx[4] + xx[5] * xx[5]);
  xx[7] = xx[6] == 0.0 ? 0.0 : xx[1] / xx[6];
  xx[8] = sin(xx[2]);
  xx[9] = xx[6] == 0.0 ? 0.0 : xx[4] / xx[6];
  xx[10] = xx[6] == 0.0 ? 0.0 : xx[5] / xx[6];
  xx[11] = xx[1] == xx[3] && xx[4] == xx[3] && xx[5] == xx[3] ? xx[0] : cos(xx[2]);
  xx[12] = xx[7] * xx[8];
  xx[13] = xx[9] * xx[8];
  xx[14] = xx[10] * xx[8];
  xx[3] = state[13];
  xx[4] = state[14];
  xx[5] = state[15];
  xx[6] = state[16];
  pm_math_Quaternion_compose_ra(xx + 11, xx + 3, xx + 7);
  state[13] = xx[7];
  state[14] = xx[8];
  state[15] = xx[9];
  state[16] = xx[10];
  state[17] = state[17] + 1.2 * mag;
  state[18] = state[18] - xx[2];
  state[19] = state[19] + 0.9 * mag;
}

static void perturbAsmJointPrimitiveState_2_0(double mag, double *state)
{
  double xx[15];
  xx[0] = 1.0;
  xx[1] = fabs(mag);
  xx[2] = xx[0] / (xx[1] - floor(xx[1]) + 1.0e-9);
  xx[1] = sin(xx[2]);
  xx[3] = 0.0;
  xx[4] = cos(xx[2]);
  xx[5] = sin(2.0 * xx[2]);
  xx[2] = 0.5 * mag;
  xx[6] = sqrt(xx[1] * xx[1] + xx[4] * xx[4] + xx[5] * xx[5]);
  xx[7] = xx[6] == 0.0 ? 0.0 : xx[1] / xx[6];
  xx[8] = sin(xx[2]);
  xx[9] = xx[6] == 0.0 ? 0.0 : xx[4] / xx[6];
  xx[10] = xx[6] == 0.0 ? 0.0 : xx[5] / xx[6];
  xx[11] = xx[1] == xx[3] && xx[4] == xx[3] && xx[5] == xx[3] ? xx[0] : cos(xx[2]);
  xx[12] = xx[7] * xx[8];
  xx[13] = xx[9] * xx[8];
  xx[14] = xx[10] * xx[8];
  xx[0] = state[20];
  xx[1] = state[21];
  xx[2] = state[22];
  xx[3] = state[23];
  pm_math_Quaternion_compose_ra(xx + 11, xx + 0, xx + 4);
  state[20] = xx[4];
  state[21] = xx[5];
  state[22] = xx[6];
  state[23] = xx[7];
}

static void perturbAsmJointPrimitiveState_2_0v(double mag, double *state)
{
  double xx[15];
  xx[0] = 1.0;
  xx[1] = fabs(mag);
  xx[2] = xx[0] / (xx[1] - floor(xx[1]) + 1.0e-9);
  xx[1] = sin(xx[2]);
  xx[3] = 0.0;
  xx[4] = cos(xx[2]);
  xx[5] = sin(2.0 * xx[2]);
  xx[2] = 0.5 * mag;
  xx[6] = sqrt(xx[1] * xx[1] + xx[4] * xx[4] + xx[5] * xx[5]);
  xx[7] = xx[6] == 0.0 ? 0.0 : xx[1] / xx[6];
  xx[8] = sin(xx[2]);
  xx[9] = xx[6] == 0.0 ? 0.0 : xx[4] / xx[6];
  xx[10] = xx[6] == 0.0 ? 0.0 : xx[5] / xx[6];
  xx[11] = xx[1] == xx[3] && xx[4] == xx[3] && xx[5] == xx[3] ? xx[0] : cos(xx[2]);
  xx[12] = xx[7] * xx[8];
  xx[13] = xx[9] * xx[8];
  xx[14] = xx[10] * xx[8];
  xx[3] = state[20];
  xx[4] = state[21];
  xx[5] = state[22];
  xx[6] = state[23];
  pm_math_Quaternion_compose_ra(xx + 11, xx + 3, xx + 7);
  state[20] = xx[7];
  state[21] = xx[8];
  state[22] = xx[9];
  state[23] = xx[10];
  state[24] = state[24] + 1.2 * mag;
  state[25] = state[25] - xx[2];
  state[26] = state[26] + 0.9 * mag;
}

static void perturbAsmJointPrimitiveState_3_0(double mag, double *state)
{
  double xx[15];
  xx[0] = 1.0;
  xx[1] = fabs(mag);
  xx[2] = xx[0] / (xx[1] - floor(xx[1]) + 1.0e-9);
  xx[1] = sin(xx[2]);
  xx[3] = 0.0;
  xx[4] = cos(xx[2]);
  xx[5] = sin(2.0 * xx[2]);
  xx[2] = 0.5 * mag;
  xx[6] = sqrt(xx[1] * xx[1] + xx[4] * xx[4] + xx[5] * xx[5]);
  xx[7] = xx[6] == 0.0 ? 0.0 : xx[1] / xx[6];
  xx[8] = sin(xx[2]);
  xx[9] = xx[6] == 0.0 ? 0.0 : xx[4] / xx[6];
  xx[10] = xx[6] == 0.0 ? 0.0 : xx[5] / xx[6];
  xx[11] = xx[1] == xx[3] && xx[4] == xx[3] && xx[5] == xx[3] ? xx[0] : cos(xx[2]);
  xx[12] = xx[7] * xx[8];
  xx[13] = xx[9] * xx[8];
  xx[14] = xx[10] * xx[8];
  xx[0] = state[27];
  xx[1] = state[28];
  xx[2] = state[29];
  xx[3] = state[30];
  pm_math_Quaternion_compose_ra(xx + 11, xx + 0, xx + 4);
  state[27] = xx[4];
  state[28] = xx[5];
  state[29] = xx[6];
  state[30] = xx[7];
}

static void perturbAsmJointPrimitiveState_3_0v(double mag, double *state)
{
  double xx[15];
  xx[0] = 1.0;
  xx[1] = fabs(mag);
  xx[2] = xx[0] / (xx[1] - floor(xx[1]) + 1.0e-9);
  xx[1] = sin(xx[2]);
  xx[3] = 0.0;
  xx[4] = cos(xx[2]);
  xx[5] = sin(2.0 * xx[2]);
  xx[2] = 0.5 * mag;
  xx[6] = sqrt(xx[1] * xx[1] + xx[4] * xx[4] + xx[5] * xx[5]);
  xx[7] = xx[6] == 0.0 ? 0.0 : xx[1] / xx[6];
  xx[8] = sin(xx[2]);
  xx[9] = xx[6] == 0.0 ? 0.0 : xx[4] / xx[6];
  xx[10] = xx[6] == 0.0 ? 0.0 : xx[5] / xx[6];
  xx[11] = xx[1] == xx[3] && xx[4] == xx[3] && xx[5] == xx[3] ? xx[0] : cos(xx[2]);
  xx[12] = xx[7] * xx[8];
  xx[13] = xx[9] * xx[8];
  xx[14] = xx[10] * xx[8];
  xx[3] = state[27];
  xx[4] = state[28];
  xx[5] = state[29];
  xx[6] = state[30];
  pm_math_Quaternion_compose_ra(xx + 11, xx + 3, xx + 7);
  state[27] = xx[7];
  state[28] = xx[8];
  state[29] = xx[9];
  state[30] = xx[10];
  state[31] = state[31] + 1.2 * mag;
  state[32] = state[32] - xx[2];
  state[33] = state[33] + 0.9 * mag;
}

static void perturbAsmJointPrimitiveState_4_0(double mag, double *state)
{
  state[34] = state[34] + mag;
}

static void perturbAsmJointPrimitiveState_4_0v(double mag, double *state)
{
  state[34] = state[34] + mag;
  state[35] = state[35] - 0.875 * mag;
}

static void perturbAsmJointPrimitiveState_5_0(double mag, double *state)
{
  state[36] = state[36] + mag;
}

static void perturbAsmJointPrimitiveState_5_0v(double mag, double *state)
{
  state[36] = state[36] + mag;
  state[37] = state[37] - 0.875 * mag;
}

static void perturbAsmJointPrimitiveState_6_0(double mag, double *state)
{
  double xx[15];
  xx[0] = 1.0;
  xx[1] = fabs(mag);
  xx[2] = xx[0] / (xx[1] - floor(xx[1]) + 1.0e-9);
  xx[1] = sin(xx[2]);
  xx[3] = 0.0;
  xx[4] = cos(xx[2]);
  xx[5] = sin(2.0 * xx[2]);
  xx[2] = 0.5 * mag;
  xx[6] = sqrt(xx[1] * xx[1] + xx[4] * xx[4] + xx[5] * xx[5]);
  xx[7] = xx[6] == 0.0 ? 0.0 : xx[1] / xx[6];
  xx[8] = sin(xx[2]);
  xx[9] = xx[6] == 0.0 ? 0.0 : xx[4] / xx[6];
  xx[10] = xx[6] == 0.0 ? 0.0 : xx[5] / xx[6];
  xx[11] = xx[1] == xx[3] && xx[4] == xx[3] && xx[5] == xx[3] ? xx[0] : cos(xx[2]);
  xx[12] = xx[7] * xx[8];
  xx[13] = xx[9] * xx[8];
  xx[14] = xx[10] * xx[8];
  xx[0] = state[38];
  xx[1] = state[39];
  xx[2] = state[40];
  xx[3] = state[41];
  pm_math_Quaternion_compose_ra(xx + 11, xx + 0, xx + 4);
  state[38] = xx[4];
  state[39] = xx[5];
  state[40] = xx[6];
  state[41] = xx[7];
}

static void perturbAsmJointPrimitiveState_6_0v(double mag, double *state)
{
  double xx[15];
  xx[0] = 1.0;
  xx[1] = fabs(mag);
  xx[2] = xx[0] / (xx[1] - floor(xx[1]) + 1.0e-9);
  xx[1] = sin(xx[2]);
  xx[3] = 0.0;
  xx[4] = cos(xx[2]);
  xx[5] = sin(2.0 * xx[2]);
  xx[2] = 0.5 * mag;
  xx[6] = sqrt(xx[1] * xx[1] + xx[4] * xx[4] + xx[5] * xx[5]);
  xx[7] = xx[6] == 0.0 ? 0.0 : xx[1] / xx[6];
  xx[8] = sin(xx[2]);
  xx[9] = xx[6] == 0.0 ? 0.0 : xx[4] / xx[6];
  xx[10] = xx[6] == 0.0 ? 0.0 : xx[5] / xx[6];
  xx[11] = xx[1] == xx[3] && xx[4] == xx[3] && xx[5] == xx[3] ? xx[0] : cos(xx[2]);
  xx[12] = xx[7] * xx[8];
  xx[13] = xx[9] * xx[8];
  xx[14] = xx[10] * xx[8];
  xx[3] = state[38];
  xx[4] = state[39];
  xx[5] = state[40];
  xx[6] = state[41];
  pm_math_Quaternion_compose_ra(xx + 11, xx + 3, xx + 7);
  state[38] = xx[7];
  state[39] = xx[8];
  state[40] = xx[9];
  state[41] = xx[10];
  state[42] = state[42] + 1.2 * mag;
  state[43] = state[43] - xx[2];
  state[44] = state[44] + 0.9 * mag;
}

static void perturbAsmJointPrimitiveState_7_0(double mag, double *state)
{
  double xx[15];
  xx[0] = 1.0;
  xx[1] = fabs(mag);
  xx[2] = xx[0] / (xx[1] - floor(xx[1]) + 1.0e-9);
  xx[1] = sin(xx[2]);
  xx[3] = 0.0;
  xx[4] = cos(xx[2]);
  xx[5] = sin(2.0 * xx[2]);
  xx[2] = 0.5 * mag;
  xx[6] = sqrt(xx[1] * xx[1] + xx[4] * xx[4] + xx[5] * xx[5]);
  xx[7] = xx[6] == 0.0 ? 0.0 : xx[1] / xx[6];
  xx[8] = sin(xx[2]);
  xx[9] = xx[6] == 0.0 ? 0.0 : xx[4] / xx[6];
  xx[10] = xx[6] == 0.0 ? 0.0 : xx[5] / xx[6];
  xx[11] = xx[1] == xx[3] && xx[4] == xx[3] && xx[5] == xx[3] ? xx[0] : cos(xx[2]);
  xx[12] = xx[7] * xx[8];
  xx[13] = xx[9] * xx[8];
  xx[14] = xx[10] * xx[8];
  xx[0] = state[45];
  xx[1] = state[46];
  xx[2] = state[47];
  xx[3] = state[48];
  pm_math_Quaternion_compose_ra(xx + 11, xx + 0, xx + 4);
  state[45] = xx[4];
  state[46] = xx[5];
  state[47] = xx[6];
  state[48] = xx[7];
}

static void perturbAsmJointPrimitiveState_7_0v(double mag, double *state)
{
  double xx[15];
  xx[0] = 1.0;
  xx[1] = fabs(mag);
  xx[2] = xx[0] / (xx[1] - floor(xx[1]) + 1.0e-9);
  xx[1] = sin(xx[2]);
  xx[3] = 0.0;
  xx[4] = cos(xx[2]);
  xx[5] = sin(2.0 * xx[2]);
  xx[2] = 0.5 * mag;
  xx[6] = sqrt(xx[1] * xx[1] + xx[4] * xx[4] + xx[5] * xx[5]);
  xx[7] = xx[6] == 0.0 ? 0.0 : xx[1] / xx[6];
  xx[8] = sin(xx[2]);
  xx[9] = xx[6] == 0.0 ? 0.0 : xx[4] / xx[6];
  xx[10] = xx[6] == 0.0 ? 0.0 : xx[5] / xx[6];
  xx[11] = xx[1] == xx[3] && xx[4] == xx[3] && xx[5] == xx[3] ? xx[0] : cos(xx[2]);
  xx[12] = xx[7] * xx[8];
  xx[13] = xx[9] * xx[8];
  xx[14] = xx[10] * xx[8];
  xx[3] = state[45];
  xx[4] = state[46];
  xx[5] = state[47];
  xx[6] = state[48];
  pm_math_Quaternion_compose_ra(xx + 11, xx + 3, xx + 7);
  state[45] = xx[7];
  state[46] = xx[8];
  state[47] = xx[9];
  state[48] = xx[10];
  state[49] = state[49] + 1.2 * mag;
  state[50] = state[50] - xx[2];
  state[51] = state[51] + 0.9 * mag;
}

static void perturbAsmJointPrimitiveState_8_0(double mag, double *state)
{
  double xx[15];
  xx[0] = 1.0;
  xx[1] = fabs(mag);
  xx[2] = xx[0] / (xx[1] - floor(xx[1]) + 1.0e-9);
  xx[1] = sin(xx[2]);
  xx[3] = 0.0;
  xx[4] = cos(xx[2]);
  xx[5] = sin(2.0 * xx[2]);
  xx[2] = 0.5 * mag;
  xx[6] = sqrt(xx[1] * xx[1] + xx[4] * xx[4] + xx[5] * xx[5]);
  xx[7] = xx[6] == 0.0 ? 0.0 : xx[1] / xx[6];
  xx[8] = sin(xx[2]);
  xx[9] = xx[6] == 0.0 ? 0.0 : xx[4] / xx[6];
  xx[10] = xx[6] == 0.0 ? 0.0 : xx[5] / xx[6];
  xx[11] = xx[1] == xx[3] && xx[4] == xx[3] && xx[5] == xx[3] ? xx[0] : cos(xx[2]);
  xx[12] = xx[7] * xx[8];
  xx[13] = xx[9] * xx[8];
  xx[14] = xx[10] * xx[8];
  xx[0] = state[52];
  xx[1] = state[53];
  xx[2] = state[54];
  xx[3] = state[55];
  pm_math_Quaternion_compose_ra(xx + 11, xx + 0, xx + 4);
  state[52] = xx[4];
  state[53] = xx[5];
  state[54] = xx[6];
  state[55] = xx[7];
}

static void perturbAsmJointPrimitiveState_8_0v(double mag, double *state)
{
  double xx[15];
  xx[0] = 1.0;
  xx[1] = fabs(mag);
  xx[2] = xx[0] / (xx[1] - floor(xx[1]) + 1.0e-9);
  xx[1] = sin(xx[2]);
  xx[3] = 0.0;
  xx[4] = cos(xx[2]);
  xx[5] = sin(2.0 * xx[2]);
  xx[2] = 0.5 * mag;
  xx[6] = sqrt(xx[1] * xx[1] + xx[4] * xx[4] + xx[5] * xx[5]);
  xx[7] = xx[6] == 0.0 ? 0.0 : xx[1] / xx[6];
  xx[8] = sin(xx[2]);
  xx[9] = xx[6] == 0.0 ? 0.0 : xx[4] / xx[6];
  xx[10] = xx[6] == 0.0 ? 0.0 : xx[5] / xx[6];
  xx[11] = xx[1] == xx[3] && xx[4] == xx[3] && xx[5] == xx[3] ? xx[0] : cos(xx[2]);
  xx[12] = xx[7] * xx[8];
  xx[13] = xx[9] * xx[8];
  xx[14] = xx[10] * xx[8];
  xx[3] = state[52];
  xx[4] = state[53];
  xx[5] = state[54];
  xx[6] = state[55];
  pm_math_Quaternion_compose_ra(xx + 11, xx + 3, xx + 7);
  state[52] = xx[7];
  state[53] = xx[8];
  state[54] = xx[9];
  state[55] = xx[10];
  state[56] = state[56] + 1.2 * mag;
  state[57] = state[57] - xx[2];
  state[58] = state[58] + 0.9 * mag;
}

static void perturbAsmJointPrimitiveState_9_0(double mag, double *state)
{
  double xx[15];
  xx[0] = 1.0;
  xx[1] = fabs(mag);
  xx[2] = xx[0] / (xx[1] - floor(xx[1]) + 1.0e-9);
  xx[1] = sin(xx[2]);
  xx[3] = 0.0;
  xx[4] = cos(xx[2]);
  xx[5] = sin(2.0 * xx[2]);
  xx[2] = 0.5 * mag;
  xx[6] = sqrt(xx[1] * xx[1] + xx[4] * xx[4] + xx[5] * xx[5]);
  xx[7] = xx[6] == 0.0 ? 0.0 : xx[1] / xx[6];
  xx[8] = sin(xx[2]);
  xx[9] = xx[6] == 0.0 ? 0.0 : xx[4] / xx[6];
  xx[10] = xx[6] == 0.0 ? 0.0 : xx[5] / xx[6];
  xx[11] = xx[1] == xx[3] && xx[4] == xx[3] && xx[5] == xx[3] ? xx[0] : cos(xx[2]);
  xx[12] = xx[7] * xx[8];
  xx[13] = xx[9] * xx[8];
  xx[14] = xx[10] * xx[8];
  xx[0] = state[59];
  xx[1] = state[60];
  xx[2] = state[61];
  xx[3] = state[62];
  pm_math_Quaternion_compose_ra(xx + 11, xx + 0, xx + 4);
  state[59] = xx[4];
  state[60] = xx[5];
  state[61] = xx[6];
  state[62] = xx[7];
}

static void perturbAsmJointPrimitiveState_9_0v(double mag, double *state)
{
  double xx[15];
  xx[0] = 1.0;
  xx[1] = fabs(mag);
  xx[2] = xx[0] / (xx[1] - floor(xx[1]) + 1.0e-9);
  xx[1] = sin(xx[2]);
  xx[3] = 0.0;
  xx[4] = cos(xx[2]);
  xx[5] = sin(2.0 * xx[2]);
  xx[2] = 0.5 * mag;
  xx[6] = sqrt(xx[1] * xx[1] + xx[4] * xx[4] + xx[5] * xx[5]);
  xx[7] = xx[6] == 0.0 ? 0.0 : xx[1] / xx[6];
  xx[8] = sin(xx[2]);
  xx[9] = xx[6] == 0.0 ? 0.0 : xx[4] / xx[6];
  xx[10] = xx[6] == 0.0 ? 0.0 : xx[5] / xx[6];
  xx[11] = xx[1] == xx[3] && xx[4] == xx[3] && xx[5] == xx[3] ? xx[0] : cos(xx[2]);
  xx[12] = xx[7] * xx[8];
  xx[13] = xx[9] * xx[8];
  xx[14] = xx[10] * xx[8];
  xx[3] = state[59];
  xx[4] = state[60];
  xx[5] = state[61];
  xx[6] = state[62];
  pm_math_Quaternion_compose_ra(xx + 11, xx + 3, xx + 7);
  state[59] = xx[7];
  state[60] = xx[8];
  state[61] = xx[9];
  state[62] = xx[10];
  state[63] = state[63] + 1.2 * mag;
  state[64] = state[64] - xx[2];
  state[65] = state[65] + 0.9 * mag;
}

static void perturbAsmJointPrimitiveState_10_0(double mag, double *state)
{
  double xx[15];
  xx[0] = 1.0;
  xx[1] = fabs(mag);
  xx[2] = xx[0] / (xx[1] - floor(xx[1]) + 1.0e-9);
  xx[1] = sin(xx[2]);
  xx[3] = 0.0;
  xx[4] = cos(xx[2]);
  xx[5] = sin(2.0 * xx[2]);
  xx[2] = 0.5 * mag;
  xx[6] = sqrt(xx[1] * xx[1] + xx[4] * xx[4] + xx[5] * xx[5]);
  xx[7] = xx[6] == 0.0 ? 0.0 : xx[1] / xx[6];
  xx[8] = sin(xx[2]);
  xx[9] = xx[6] == 0.0 ? 0.0 : xx[4] / xx[6];
  xx[10] = xx[6] == 0.0 ? 0.0 : xx[5] / xx[6];
  xx[11] = xx[1] == xx[3] && xx[4] == xx[3] && xx[5] == xx[3] ? xx[0] : cos(xx[2]);
  xx[12] = xx[7] * xx[8];
  xx[13] = xx[9] * xx[8];
  xx[14] = xx[10] * xx[8];
  xx[0] = state[66];
  xx[1] = state[67];
  xx[2] = state[68];
  xx[3] = state[69];
  pm_math_Quaternion_compose_ra(xx + 11, xx + 0, xx + 4);
  state[66] = xx[4];
  state[67] = xx[5];
  state[68] = xx[6];
  state[69] = xx[7];
}

static void perturbAsmJointPrimitiveState_10_0v(double mag, double *state)
{
  double xx[15];
  xx[0] = 1.0;
  xx[1] = fabs(mag);
  xx[2] = xx[0] / (xx[1] - floor(xx[1]) + 1.0e-9);
  xx[1] = sin(xx[2]);
  xx[3] = 0.0;
  xx[4] = cos(xx[2]);
  xx[5] = sin(2.0 * xx[2]);
  xx[2] = 0.5 * mag;
  xx[6] = sqrt(xx[1] * xx[1] + xx[4] * xx[4] + xx[5] * xx[5]);
  xx[7] = xx[6] == 0.0 ? 0.0 : xx[1] / xx[6];
  xx[8] = sin(xx[2]);
  xx[9] = xx[6] == 0.0 ? 0.0 : xx[4] / xx[6];
  xx[10] = xx[6] == 0.0 ? 0.0 : xx[5] / xx[6];
  xx[11] = xx[1] == xx[3] && xx[4] == xx[3] && xx[5] == xx[3] ? xx[0] : cos(xx[2]);
  xx[12] = xx[7] * xx[8];
  xx[13] = xx[9] * xx[8];
  xx[14] = xx[10] * xx[8];
  xx[3] = state[66];
  xx[4] = state[67];
  xx[5] = state[68];
  xx[6] = state[69];
  pm_math_Quaternion_compose_ra(xx + 11, xx + 3, xx + 7);
  state[66] = xx[7];
  state[67] = xx[8];
  state[68] = xx[9];
  state[69] = xx[10];
  state[70] = state[70] + 1.2 * mag;
  state[71] = state[71] - xx[2];
  state[72] = state[72] + 0.9 * mag;
}

static void perturbAsmJointPrimitiveState_11_0(double mag, double *state)
{
  double xx[15];
  xx[0] = 1.0;
  xx[1] = fabs(mag);
  xx[2] = xx[0] / (xx[1] - floor(xx[1]) + 1.0e-9);
  xx[1] = sin(xx[2]);
  xx[3] = 0.0;
  xx[4] = cos(xx[2]);
  xx[5] = sin(2.0 * xx[2]);
  xx[2] = 0.5 * mag;
  xx[6] = sqrt(xx[1] * xx[1] + xx[4] * xx[4] + xx[5] * xx[5]);
  xx[7] = xx[6] == 0.0 ? 0.0 : xx[1] / xx[6];
  xx[8] = sin(xx[2]);
  xx[9] = xx[6] == 0.0 ? 0.0 : xx[4] / xx[6];
  xx[10] = xx[6] == 0.0 ? 0.0 : xx[5] / xx[6];
  xx[11] = xx[1] == xx[3] && xx[4] == xx[3] && xx[5] == xx[3] ? xx[0] : cos(xx[2]);
  xx[12] = xx[7] * xx[8];
  xx[13] = xx[9] * xx[8];
  xx[14] = xx[10] * xx[8];
  xx[0] = state[73];
  xx[1] = state[74];
  xx[2] = state[75];
  xx[3] = state[76];
  pm_math_Quaternion_compose_ra(xx + 11, xx + 0, xx + 4);
  state[73] = xx[4];
  state[74] = xx[5];
  state[75] = xx[6];
  state[76] = xx[7];
}

static void perturbAsmJointPrimitiveState_11_0v(double mag, double *state)
{
  double xx[15];
  xx[0] = 1.0;
  xx[1] = fabs(mag);
  xx[2] = xx[0] / (xx[1] - floor(xx[1]) + 1.0e-9);
  xx[1] = sin(xx[2]);
  xx[3] = 0.0;
  xx[4] = cos(xx[2]);
  xx[5] = sin(2.0 * xx[2]);
  xx[2] = 0.5 * mag;
  xx[6] = sqrt(xx[1] * xx[1] + xx[4] * xx[4] + xx[5] * xx[5]);
  xx[7] = xx[6] == 0.0 ? 0.0 : xx[1] / xx[6];
  xx[8] = sin(xx[2]);
  xx[9] = xx[6] == 0.0 ? 0.0 : xx[4] / xx[6];
  xx[10] = xx[6] == 0.0 ? 0.0 : xx[5] / xx[6];
  xx[11] = xx[1] == xx[3] && xx[4] == xx[3] && xx[5] == xx[3] ? xx[0] : cos(xx[2]);
  xx[12] = xx[7] * xx[8];
  xx[13] = xx[9] * xx[8];
  xx[14] = xx[10] * xx[8];
  xx[3] = state[73];
  xx[4] = state[74];
  xx[5] = state[75];
  xx[6] = state[76];
  pm_math_Quaternion_compose_ra(xx + 11, xx + 3, xx + 7);
  state[73] = xx[7];
  state[74] = xx[8];
  state[75] = xx[9];
  state[76] = xx[10];
  state[77] = state[77] + 1.2 * mag;
  state[78] = state[78] - xx[2];
  state[79] = state[79] + 0.9 * mag;
}

static void perturbAsmJointPrimitiveState_12_0(double mag, double *state)
{
  double xx[15];
  xx[0] = 1.0;
  xx[1] = fabs(mag);
  xx[2] = xx[0] / (xx[1] - floor(xx[1]) + 1.0e-9);
  xx[1] = sin(xx[2]);
  xx[3] = 0.0;
  xx[4] = cos(xx[2]);
  xx[5] = sin(2.0 * xx[2]);
  xx[2] = 0.5 * mag;
  xx[6] = sqrt(xx[1] * xx[1] + xx[4] * xx[4] + xx[5] * xx[5]);
  xx[7] = xx[6] == 0.0 ? 0.0 : xx[1] / xx[6];
  xx[8] = sin(xx[2]);
  xx[9] = xx[6] == 0.0 ? 0.0 : xx[4] / xx[6];
  xx[10] = xx[6] == 0.0 ? 0.0 : xx[5] / xx[6];
  xx[11] = xx[1] == xx[3] && xx[4] == xx[3] && xx[5] == xx[3] ? xx[0] : cos(xx[2]);
  xx[12] = xx[7] * xx[8];
  xx[13] = xx[9] * xx[8];
  xx[14] = xx[10] * xx[8];
  xx[0] = state[80];
  xx[1] = state[81];
  xx[2] = state[82];
  xx[3] = state[83];
  pm_math_Quaternion_compose_ra(xx + 11, xx + 0, xx + 4);
  state[80] = xx[4];
  state[81] = xx[5];
  state[82] = xx[6];
  state[83] = xx[7];
}

static void perturbAsmJointPrimitiveState_12_0v(double mag, double *state)
{
  double xx[15];
  xx[0] = 1.0;
  xx[1] = fabs(mag);
  xx[2] = xx[0] / (xx[1] - floor(xx[1]) + 1.0e-9);
  xx[1] = sin(xx[2]);
  xx[3] = 0.0;
  xx[4] = cos(xx[2]);
  xx[5] = sin(2.0 * xx[2]);
  xx[2] = 0.5 * mag;
  xx[6] = sqrt(xx[1] * xx[1] + xx[4] * xx[4] + xx[5] * xx[5]);
  xx[7] = xx[6] == 0.0 ? 0.0 : xx[1] / xx[6];
  xx[8] = sin(xx[2]);
  xx[9] = xx[6] == 0.0 ? 0.0 : xx[4] / xx[6];
  xx[10] = xx[6] == 0.0 ? 0.0 : xx[5] / xx[6];
  xx[11] = xx[1] == xx[3] && xx[4] == xx[3] && xx[5] == xx[3] ? xx[0] : cos(xx[2]);
  xx[12] = xx[7] * xx[8];
  xx[13] = xx[9] * xx[8];
  xx[14] = xx[10] * xx[8];
  xx[3] = state[80];
  xx[4] = state[81];
  xx[5] = state[82];
  xx[6] = state[83];
  pm_math_Quaternion_compose_ra(xx + 11, xx + 3, xx + 7);
  state[80] = xx[7];
  state[81] = xx[8];
  state[82] = xx[9];
  state[83] = xx[10];
  state[84] = state[84] + 1.2 * mag;
  state[85] = state[85] - xx[2];
  state[86] = state[86] + 0.9 * mag;
}

static void perturbAsmJointPrimitiveState_13_0(double mag, double *state)
{
  double xx[15];
  xx[0] = 1.0;
  xx[1] = fabs(mag);
  xx[2] = xx[0] / (xx[1] - floor(xx[1]) + 1.0e-9);
  xx[1] = sin(xx[2]);
  xx[3] = 0.0;
  xx[4] = cos(xx[2]);
  xx[5] = sin(2.0 * xx[2]);
  xx[2] = 0.5 * mag;
  xx[6] = sqrt(xx[1] * xx[1] + xx[4] * xx[4] + xx[5] * xx[5]);
  xx[7] = xx[6] == 0.0 ? 0.0 : xx[1] / xx[6];
  xx[8] = sin(xx[2]);
  xx[9] = xx[6] == 0.0 ? 0.0 : xx[4] / xx[6];
  xx[10] = xx[6] == 0.0 ? 0.0 : xx[5] / xx[6];
  xx[11] = xx[1] == xx[3] && xx[4] == xx[3] && xx[5] == xx[3] ? xx[0] : cos(xx[2]);
  xx[12] = xx[7] * xx[8];
  xx[13] = xx[9] * xx[8];
  xx[14] = xx[10] * xx[8];
  xx[0] = state[87];
  xx[1] = state[88];
  xx[2] = state[89];
  xx[3] = state[90];
  pm_math_Quaternion_compose_ra(xx + 11, xx + 0, xx + 4);
  state[87] = xx[4];
  state[88] = xx[5];
  state[89] = xx[6];
  state[90] = xx[7];
}

static void perturbAsmJointPrimitiveState_13_0v(double mag, double *state)
{
  double xx[15];
  xx[0] = 1.0;
  xx[1] = fabs(mag);
  xx[2] = xx[0] / (xx[1] - floor(xx[1]) + 1.0e-9);
  xx[1] = sin(xx[2]);
  xx[3] = 0.0;
  xx[4] = cos(xx[2]);
  xx[5] = sin(2.0 * xx[2]);
  xx[2] = 0.5 * mag;
  xx[6] = sqrt(xx[1] * xx[1] + xx[4] * xx[4] + xx[5] * xx[5]);
  xx[7] = xx[6] == 0.0 ? 0.0 : xx[1] / xx[6];
  xx[8] = sin(xx[2]);
  xx[9] = xx[6] == 0.0 ? 0.0 : xx[4] / xx[6];
  xx[10] = xx[6] == 0.0 ? 0.0 : xx[5] / xx[6];
  xx[11] = xx[1] == xx[3] && xx[4] == xx[3] && xx[5] == xx[3] ? xx[0] : cos(xx[2]);
  xx[12] = xx[7] * xx[8];
  xx[13] = xx[9] * xx[8];
  xx[14] = xx[10] * xx[8];
  xx[3] = state[87];
  xx[4] = state[88];
  xx[5] = state[89];
  xx[6] = state[90];
  pm_math_Quaternion_compose_ra(xx + 11, xx + 3, xx + 7);
  state[87] = xx[7];
  state[88] = xx[8];
  state[89] = xx[9];
  state[90] = xx[10];
  state[91] = state[91] + 1.2 * mag;
  state[92] = state[92] - xx[2];
  state[93] = state[93] + 0.9 * mag;
}

static void perturbAsmJointPrimitiveState_14_0(double mag, double *state)
{
  state[94] = state[94] + mag;
}

static void perturbAsmJointPrimitiveState_14_0v(double mag, double *state)
{
  state[94] = state[94] + mag;
  state[95] = state[95] - 0.875 * mag;
}

static void perturbAsmJointPrimitiveState_15_0(double mag, double *state)
{
  state[96] = state[96] + mag;
}

static void perturbAsmJointPrimitiveState_15_0v(double mag, double *state)
{
  state[96] = state[96] + mag;
  state[97] = state[97] - 0.875 * mag;
}

static void perturbAsmJointPrimitiveState_16_0(double mag, double *state)
{
  double xx[15];
  xx[0] = 1.0;
  xx[1] = fabs(mag);
  xx[2] = xx[0] / (xx[1] - floor(xx[1]) + 1.0e-9);
  xx[1] = sin(xx[2]);
  xx[3] = 0.0;
  xx[4] = cos(xx[2]);
  xx[5] = sin(2.0 * xx[2]);
  xx[2] = 0.5 * mag;
  xx[6] = sqrt(xx[1] * xx[1] + xx[4] * xx[4] + xx[5] * xx[5]);
  xx[7] = xx[6] == 0.0 ? 0.0 : xx[1] / xx[6];
  xx[8] = sin(xx[2]);
  xx[9] = xx[6] == 0.0 ? 0.0 : xx[4] / xx[6];
  xx[10] = xx[6] == 0.0 ? 0.0 : xx[5] / xx[6];
  xx[11] = xx[1] == xx[3] && xx[4] == xx[3] && xx[5] == xx[3] ? xx[0] : cos(xx[2]);
  xx[12] = xx[7] * xx[8];
  xx[13] = xx[9] * xx[8];
  xx[14] = xx[10] * xx[8];
  xx[0] = state[98];
  xx[1] = state[99];
  xx[2] = state[100];
  xx[3] = state[101];
  pm_math_Quaternion_compose_ra(xx + 11, xx + 0, xx + 4);
  state[98] = xx[4];
  state[99] = xx[5];
  state[100] = xx[6];
  state[101] = xx[7];
}

static void perturbAsmJointPrimitiveState_16_0v(double mag, double *state)
{
  double xx[15];
  xx[0] = 1.0;
  xx[1] = fabs(mag);
  xx[2] = xx[0] / (xx[1] - floor(xx[1]) + 1.0e-9);
  xx[1] = sin(xx[2]);
  xx[3] = 0.0;
  xx[4] = cos(xx[2]);
  xx[5] = sin(2.0 * xx[2]);
  xx[2] = 0.5 * mag;
  xx[6] = sqrt(xx[1] * xx[1] + xx[4] * xx[4] + xx[5] * xx[5]);
  xx[7] = xx[6] == 0.0 ? 0.0 : xx[1] / xx[6];
  xx[8] = sin(xx[2]);
  xx[9] = xx[6] == 0.0 ? 0.0 : xx[4] / xx[6];
  xx[10] = xx[6] == 0.0 ? 0.0 : xx[5] / xx[6];
  xx[11] = xx[1] == xx[3] && xx[4] == xx[3] && xx[5] == xx[3] ? xx[0] : cos(xx[2]);
  xx[12] = xx[7] * xx[8];
  xx[13] = xx[9] * xx[8];
  xx[14] = xx[10] * xx[8];
  xx[3] = state[98];
  xx[4] = state[99];
  xx[5] = state[100];
  xx[6] = state[101];
  pm_math_Quaternion_compose_ra(xx + 11, xx + 3, xx + 7);
  state[98] = xx[7];
  state[99] = xx[8];
  state[100] = xx[9];
  state[101] = xx[10];
  state[102] = state[102] + 1.2 * mag;
  state[103] = state[103] - xx[2];
  state[104] = state[104] + 0.9 * mag;
}

static void perturbAsmJointPrimitiveState_17_0(double mag, double *state)
{
  double xx[15];
  xx[0] = 1.0;
  xx[1] = fabs(mag);
  xx[2] = xx[0] / (xx[1] - floor(xx[1]) + 1.0e-9);
  xx[1] = sin(xx[2]);
  xx[3] = 0.0;
  xx[4] = cos(xx[2]);
  xx[5] = sin(2.0 * xx[2]);
  xx[2] = 0.5 * mag;
  xx[6] = sqrt(xx[1] * xx[1] + xx[4] * xx[4] + xx[5] * xx[5]);
  xx[7] = xx[6] == 0.0 ? 0.0 : xx[1] / xx[6];
  xx[8] = sin(xx[2]);
  xx[9] = xx[6] == 0.0 ? 0.0 : xx[4] / xx[6];
  xx[10] = xx[6] == 0.0 ? 0.0 : xx[5] / xx[6];
  xx[11] = xx[1] == xx[3] && xx[4] == xx[3] && xx[5] == xx[3] ? xx[0] : cos(xx[2]);
  xx[12] = xx[7] * xx[8];
  xx[13] = xx[9] * xx[8];
  xx[14] = xx[10] * xx[8];
  xx[0] = state[105];
  xx[1] = state[106];
  xx[2] = state[107];
  xx[3] = state[108];
  pm_math_Quaternion_compose_ra(xx + 11, xx + 0, xx + 4);
  state[105] = xx[4];
  state[106] = xx[5];
  state[107] = xx[6];
  state[108] = xx[7];
}

static void perturbAsmJointPrimitiveState_17_0v(double mag, double *state)
{
  double xx[15];
  xx[0] = 1.0;
  xx[1] = fabs(mag);
  xx[2] = xx[0] / (xx[1] - floor(xx[1]) + 1.0e-9);
  xx[1] = sin(xx[2]);
  xx[3] = 0.0;
  xx[4] = cos(xx[2]);
  xx[5] = sin(2.0 * xx[2]);
  xx[2] = 0.5 * mag;
  xx[6] = sqrt(xx[1] * xx[1] + xx[4] * xx[4] + xx[5] * xx[5]);
  xx[7] = xx[6] == 0.0 ? 0.0 : xx[1] / xx[6];
  xx[8] = sin(xx[2]);
  xx[9] = xx[6] == 0.0 ? 0.0 : xx[4] / xx[6];
  xx[10] = xx[6] == 0.0 ? 0.0 : xx[5] / xx[6];
  xx[11] = xx[1] == xx[3] && xx[4] == xx[3] && xx[5] == xx[3] ? xx[0] : cos(xx[2]);
  xx[12] = xx[7] * xx[8];
  xx[13] = xx[9] * xx[8];
  xx[14] = xx[10] * xx[8];
  xx[3] = state[105];
  xx[4] = state[106];
  xx[5] = state[107];
  xx[6] = state[108];
  pm_math_Quaternion_compose_ra(xx + 11, xx + 3, xx + 7);
  state[105] = xx[7];
  state[106] = xx[8];
  state[107] = xx[9];
  state[108] = xx[10];
  state[109] = state[109] + 1.2 * mag;
  state[110] = state[110] - xx[2];
  state[111] = state[111] + 0.9 * mag;
}

static void perturbAsmJointPrimitiveState_18_0(double mag, double *state)
{
  double xx[15];
  xx[0] = 1.0;
  xx[1] = fabs(mag);
  xx[2] = xx[0] / (xx[1] - floor(xx[1]) + 1.0e-9);
  xx[1] = sin(xx[2]);
  xx[3] = 0.0;
  xx[4] = cos(xx[2]);
  xx[5] = sin(2.0 * xx[2]);
  xx[2] = 0.5 * mag;
  xx[6] = sqrt(xx[1] * xx[1] + xx[4] * xx[4] + xx[5] * xx[5]);
  xx[7] = xx[6] == 0.0 ? 0.0 : xx[1] / xx[6];
  xx[8] = sin(xx[2]);
  xx[9] = xx[6] == 0.0 ? 0.0 : xx[4] / xx[6];
  xx[10] = xx[6] == 0.0 ? 0.0 : xx[5] / xx[6];
  xx[11] = xx[1] == xx[3] && xx[4] == xx[3] && xx[5] == xx[3] ? xx[0] : cos(xx[2]);
  xx[12] = xx[7] * xx[8];
  xx[13] = xx[9] * xx[8];
  xx[14] = xx[10] * xx[8];
  xx[0] = state[112];
  xx[1] = state[113];
  xx[2] = state[114];
  xx[3] = state[115];
  pm_math_Quaternion_compose_ra(xx + 11, xx + 0, xx + 4);
  state[112] = xx[4];
  state[113] = xx[5];
  state[114] = xx[6];
  state[115] = xx[7];
}

static void perturbAsmJointPrimitiveState_18_0v(double mag, double *state)
{
  double xx[15];
  xx[0] = 1.0;
  xx[1] = fabs(mag);
  xx[2] = xx[0] / (xx[1] - floor(xx[1]) + 1.0e-9);
  xx[1] = sin(xx[2]);
  xx[3] = 0.0;
  xx[4] = cos(xx[2]);
  xx[5] = sin(2.0 * xx[2]);
  xx[2] = 0.5 * mag;
  xx[6] = sqrt(xx[1] * xx[1] + xx[4] * xx[4] + xx[5] * xx[5]);
  xx[7] = xx[6] == 0.0 ? 0.0 : xx[1] / xx[6];
  xx[8] = sin(xx[2]);
  xx[9] = xx[6] == 0.0 ? 0.0 : xx[4] / xx[6];
  xx[10] = xx[6] == 0.0 ? 0.0 : xx[5] / xx[6];
  xx[11] = xx[1] == xx[3] && xx[4] == xx[3] && xx[5] == xx[3] ? xx[0] : cos(xx[2]);
  xx[12] = xx[7] * xx[8];
  xx[13] = xx[9] * xx[8];
  xx[14] = xx[10] * xx[8];
  xx[3] = state[112];
  xx[4] = state[113];
  xx[5] = state[114];
  xx[6] = state[115];
  pm_math_Quaternion_compose_ra(xx + 11, xx + 3, xx + 7);
  state[112] = xx[7];
  state[113] = xx[8];
  state[114] = xx[9];
  state[115] = xx[10];
  state[116] = state[116] + 1.2 * mag;
  state[117] = state[117] - xx[2];
  state[118] = state[118] + 0.9 * mag;
}

static void perturbAsmJointPrimitiveState_19_0(double mag, double *state)
{
  double xx[15];
  xx[0] = 1.0;
  xx[1] = fabs(mag);
  xx[2] = xx[0] / (xx[1] - floor(xx[1]) + 1.0e-9);
  xx[1] = sin(xx[2]);
  xx[3] = 0.0;
  xx[4] = cos(xx[2]);
  xx[5] = sin(2.0 * xx[2]);
  xx[2] = 0.5 * mag;
  xx[6] = sqrt(xx[1] * xx[1] + xx[4] * xx[4] + xx[5] * xx[5]);
  xx[7] = xx[6] == 0.0 ? 0.0 : xx[1] / xx[6];
  xx[8] = sin(xx[2]);
  xx[9] = xx[6] == 0.0 ? 0.0 : xx[4] / xx[6];
  xx[10] = xx[6] == 0.0 ? 0.0 : xx[5] / xx[6];
  xx[11] = xx[1] == xx[3] && xx[4] == xx[3] && xx[5] == xx[3] ? xx[0] : cos(xx[2]);
  xx[12] = xx[7] * xx[8];
  xx[13] = xx[9] * xx[8];
  xx[14] = xx[10] * xx[8];
  xx[0] = state[119];
  xx[1] = state[120];
  xx[2] = state[121];
  xx[3] = state[122];
  pm_math_Quaternion_compose_ra(xx + 11, xx + 0, xx + 4);
  state[119] = xx[4];
  state[120] = xx[5];
  state[121] = xx[6];
  state[122] = xx[7];
}

static void perturbAsmJointPrimitiveState_19_0v(double mag, double *state)
{
  double xx[15];
  xx[0] = 1.0;
  xx[1] = fabs(mag);
  xx[2] = xx[0] / (xx[1] - floor(xx[1]) + 1.0e-9);
  xx[1] = sin(xx[2]);
  xx[3] = 0.0;
  xx[4] = cos(xx[2]);
  xx[5] = sin(2.0 * xx[2]);
  xx[2] = 0.5 * mag;
  xx[6] = sqrt(xx[1] * xx[1] + xx[4] * xx[4] + xx[5] * xx[5]);
  xx[7] = xx[6] == 0.0 ? 0.0 : xx[1] / xx[6];
  xx[8] = sin(xx[2]);
  xx[9] = xx[6] == 0.0 ? 0.0 : xx[4] / xx[6];
  xx[10] = xx[6] == 0.0 ? 0.0 : xx[5] / xx[6];
  xx[11] = xx[1] == xx[3] && xx[4] == xx[3] && xx[5] == xx[3] ? xx[0] : cos(xx[2]);
  xx[12] = xx[7] * xx[8];
  xx[13] = xx[9] * xx[8];
  xx[14] = xx[10] * xx[8];
  xx[3] = state[119];
  xx[4] = state[120];
  xx[5] = state[121];
  xx[6] = state[122];
  pm_math_Quaternion_compose_ra(xx + 11, xx + 3, xx + 7);
  state[119] = xx[7];
  state[120] = xx[8];
  state[121] = xx[9];
  state[122] = xx[10];
  state[123] = state[123] + 1.2 * mag;
  state[124] = state[124] - xx[2];
  state[125] = state[125] + 0.9 * mag;
}

static void perturbAsmJointPrimitiveState_20_0(double mag, double *state)
{
  double xx[15];
  xx[0] = 1.0;
  xx[1] = fabs(mag);
  xx[2] = xx[0] / (xx[1] - floor(xx[1]) + 1.0e-9);
  xx[1] = sin(xx[2]);
  xx[3] = 0.0;
  xx[4] = cos(xx[2]);
  xx[5] = sin(2.0 * xx[2]);
  xx[2] = 0.5 * mag;
  xx[6] = sqrt(xx[1] * xx[1] + xx[4] * xx[4] + xx[5] * xx[5]);
  xx[7] = xx[6] == 0.0 ? 0.0 : xx[1] / xx[6];
  xx[8] = sin(xx[2]);
  xx[9] = xx[6] == 0.0 ? 0.0 : xx[4] / xx[6];
  xx[10] = xx[6] == 0.0 ? 0.0 : xx[5] / xx[6];
  xx[11] = xx[1] == xx[3] && xx[4] == xx[3] && xx[5] == xx[3] ? xx[0] : cos(xx[2]);
  xx[12] = xx[7] * xx[8];
  xx[13] = xx[9] * xx[8];
  xx[14] = xx[10] * xx[8];
  xx[0] = state[126];
  xx[1] = state[127];
  xx[2] = state[128];
  xx[3] = state[129];
  pm_math_Quaternion_compose_ra(xx + 11, xx + 0, xx + 4);
  state[126] = xx[4];
  state[127] = xx[5];
  state[128] = xx[6];
  state[129] = xx[7];
}

static void perturbAsmJointPrimitiveState_20_0v(double mag, double *state)
{
  double xx[15];
  xx[0] = 1.0;
  xx[1] = fabs(mag);
  xx[2] = xx[0] / (xx[1] - floor(xx[1]) + 1.0e-9);
  xx[1] = sin(xx[2]);
  xx[3] = 0.0;
  xx[4] = cos(xx[2]);
  xx[5] = sin(2.0 * xx[2]);
  xx[2] = 0.5 * mag;
  xx[6] = sqrt(xx[1] * xx[1] + xx[4] * xx[4] + xx[5] * xx[5]);
  xx[7] = xx[6] == 0.0 ? 0.0 : xx[1] / xx[6];
  xx[8] = sin(xx[2]);
  xx[9] = xx[6] == 0.0 ? 0.0 : xx[4] / xx[6];
  xx[10] = xx[6] == 0.0 ? 0.0 : xx[5] / xx[6];
  xx[11] = xx[1] == xx[3] && xx[4] == xx[3] && xx[5] == xx[3] ? xx[0] : cos(xx[2]);
  xx[12] = xx[7] * xx[8];
  xx[13] = xx[9] * xx[8];
  xx[14] = xx[10] * xx[8];
  xx[3] = state[126];
  xx[4] = state[127];
  xx[5] = state[128];
  xx[6] = state[129];
  pm_math_Quaternion_compose_ra(xx + 11, xx + 3, xx + 7);
  state[126] = xx[7];
  state[127] = xx[8];
  state[128] = xx[9];
  state[129] = xx[10];
  state[130] = state[130] + 1.2 * mag;
  state[131] = state[131] - xx[2];
  state[132] = state[132] + 0.9 * mag;
}

void dempsystest_bjorn_59d2bdba_5_perturbAsmJointPrimitiveState(const void *mech,
  size_t stageIdx, size_t primIdx, double mag, boolean_T doPerturbVelocity,
  double *state)
{
  (void) mech;
  (void) stageIdx;
  (void) primIdx;
  (void) mag;
  (void) doPerturbVelocity;
  (void) state;
  switch ((stageIdx * 6 + primIdx) * 2 + (doPerturbVelocity ? 1 : 0))
  {
   case 0:
    perturbAsmJointPrimitiveState_0_0(mag, state);
    break;

   case 1:
    perturbAsmJointPrimitiveState_0_0v(mag, state);
    break;

   case 2:
    perturbAsmJointPrimitiveState_0_1(mag, state);
    break;

   case 3:
    perturbAsmJointPrimitiveState_0_1v(mag, state);
    break;

   case 4:
    perturbAsmJointPrimitiveState_0_2(mag, state);
    break;

   case 5:
    perturbAsmJointPrimitiveState_0_2v(mag, state);
    break;

   case 6:
    perturbAsmJointPrimitiveState_0_3(mag, state);
    break;

   case 7:
    perturbAsmJointPrimitiveState_0_3v(mag, state);
    break;

   case 12:
    perturbAsmJointPrimitiveState_1_0(mag, state);
    break;

   case 13:
    perturbAsmJointPrimitiveState_1_0v(mag, state);
    break;

   case 24:
    perturbAsmJointPrimitiveState_2_0(mag, state);
    break;

   case 25:
    perturbAsmJointPrimitiveState_2_0v(mag, state);
    break;

   case 36:
    perturbAsmJointPrimitiveState_3_0(mag, state);
    break;

   case 37:
    perturbAsmJointPrimitiveState_3_0v(mag, state);
    break;

   case 48:
    perturbAsmJointPrimitiveState_4_0(mag, state);
    break;

   case 49:
    perturbAsmJointPrimitiveState_4_0v(mag, state);
    break;

   case 60:
    perturbAsmJointPrimitiveState_5_0(mag, state);
    break;

   case 61:
    perturbAsmJointPrimitiveState_5_0v(mag, state);
    break;

   case 72:
    perturbAsmJointPrimitiveState_6_0(mag, state);
    break;

   case 73:
    perturbAsmJointPrimitiveState_6_0v(mag, state);
    break;

   case 84:
    perturbAsmJointPrimitiveState_7_0(mag, state);
    break;

   case 85:
    perturbAsmJointPrimitiveState_7_0v(mag, state);
    break;

   case 96:
    perturbAsmJointPrimitiveState_8_0(mag, state);
    break;

   case 97:
    perturbAsmJointPrimitiveState_8_0v(mag, state);
    break;

   case 108:
    perturbAsmJointPrimitiveState_9_0(mag, state);
    break;

   case 109:
    perturbAsmJointPrimitiveState_9_0v(mag, state);
    break;

   case 120:
    perturbAsmJointPrimitiveState_10_0(mag, state);
    break;

   case 121:
    perturbAsmJointPrimitiveState_10_0v(mag, state);
    break;

   case 132:
    perturbAsmJointPrimitiveState_11_0(mag, state);
    break;

   case 133:
    perturbAsmJointPrimitiveState_11_0v(mag, state);
    break;

   case 144:
    perturbAsmJointPrimitiveState_12_0(mag, state);
    break;

   case 145:
    perturbAsmJointPrimitiveState_12_0v(mag, state);
    break;

   case 156:
    perturbAsmJointPrimitiveState_13_0(mag, state);
    break;

   case 157:
    perturbAsmJointPrimitiveState_13_0v(mag, state);
    break;

   case 168:
    perturbAsmJointPrimitiveState_14_0(mag, state);
    break;

   case 169:
    perturbAsmJointPrimitiveState_14_0v(mag, state);
    break;

   case 180:
    perturbAsmJointPrimitiveState_15_0(mag, state);
    break;

   case 181:
    perturbAsmJointPrimitiveState_15_0v(mag, state);
    break;

   case 192:
    perturbAsmJointPrimitiveState_16_0(mag, state);
    break;

   case 193:
    perturbAsmJointPrimitiveState_16_0v(mag, state);
    break;

   case 204:
    perturbAsmJointPrimitiveState_17_0(mag, state);
    break;

   case 205:
    perturbAsmJointPrimitiveState_17_0v(mag, state);
    break;

   case 216:
    perturbAsmJointPrimitiveState_18_0(mag, state);
    break;

   case 217:
    perturbAsmJointPrimitiveState_18_0v(mag, state);
    break;

   case 228:
    perturbAsmJointPrimitiveState_19_0(mag, state);
    break;

   case 229:
    perturbAsmJointPrimitiveState_19_0v(mag, state);
    break;

   case 240:
    perturbAsmJointPrimitiveState_20_0(mag, state);
    break;

   case 241:
    perturbAsmJointPrimitiveState_20_0v(mag, state);
    break;
  }
}

static void computePosDofBlendMatrix_0_3(const double *state, int partialType,
  double *matrix)
{
  double xx[20];
  xx[0] = 0.0;
  xx[1] = 2.0;
  xx[2] = xx[1] * (state[4] * state[5] - state[3] * state[6]);
  xx[3] = xx[2] * xx[2];
  xx[4] = 1.0;
  xx[5] = (state[3] * state[3] + state[4] * state[4]) * xx[1] - xx[4];
  xx[6] = xx[5] * xx[5];
  xx[7] = sqrt(xx[3] + xx[6]);
  xx[8] = xx[7] == 0.0 ? 0.0 : - xx[2] / xx[7];
  xx[9] = xx[6] + xx[3];
  xx[3] = sqrt(xx[9]);
  xx[6] = xx[3] == 0.0 ? 0.0 : xx[5] / xx[3];
  xx[10] = 0.0;
  xx[11] = (state[4] * state[6] + state[3] * state[5]) * xx[1];
  xx[1] = sqrt(xx[9] + xx[11] * xx[11]);
  xx[12] = xx[1] == 0.0 ? 0.0 : xx[5] / xx[1];
  xx[14] = xx[8];
  xx[15] = xx[6];
  xx[16] = xx[10];
  xx[17] = xx[8];
  xx[18] = xx[8];
  xx[19] = xx[12];
  xx[6] = xx[13 + (partialType)];
  xx[8] = xx[7] == 0.0 ? 0.0 : xx[5] / xx[7];
  xx[7] = xx[3] == 0.0 ? 0.0 : xx[2] / xx[3];
  xx[3] = xx[1] == 0.0 ? 0.0 : xx[2] / xx[1];
  xx[13] = xx[8];
  xx[14] = xx[7];
  xx[15] = xx[10];
  xx[16] = xx[8];
  xx[17] = xx[8];
  xx[18] = xx[3];
  xx[2] = xx[12 + (partialType)];
  xx[3] = xx[1] == 0.0 ? 0.0 : xx[11] / xx[1];
  xx[13] = xx[10];
  xx[14] = xx[10];
  xx[15] = xx[4];
  xx[16] = xx[10];
  xx[17] = xx[10];
  xx[18] = xx[3];
  xx[1] = xx[12 + (partialType)];
  xx[3] = xx[11] * xx[5];
  xx[5] = sqrt(xx[9] * xx[9] + xx[3] * xx[3]);
  xx[7] = xx[5] == 0.0 ? 0.0 : xx[9] / xx[5];
  xx[12] = xx[10];
  xx[13] = xx[10];
  xx[14] = xx[10];
  xx[15] = xx[7];
  xx[16] = xx[10];
  xx[17] = xx[10];
  xx[7] = xx[11 + (partialType)];
  xx[12] = xx[10];
  xx[13] = xx[10];
  xx[14] = xx[10];
  xx[15] = xx[10];
  xx[16] = xx[10];
  xx[17] = xx[10];
  xx[8] = xx[11 + (partialType)];
  xx[9] = xx[5] == 0.0 ? 0.0 : xx[3] / xx[5];
  xx[12] = xx[4];
  xx[13] = xx[4];
  xx[14] = xx[10];
  xx[15] = xx[9];
  xx[16] = xx[10];
  xx[17] = xx[10];
  xx[0] = xx[11 + (partialType)];
  matrix[0] = xx[6];
  matrix[1] = xx[2];
  matrix[2] = xx[1];
  matrix[3] = xx[7];
  matrix[4] = xx[8];
  matrix[5] = xx[0];
  matrix[6] = xx[8];
  matrix[7] = xx[8];
  matrix[8] = xx[8];
}

static void computePosDofBlendMatrix_1_0(const double *state, int partialType,
  double *matrix)
{
  double xx[20];
  xx[0] = 0.0;
  xx[1] = 2.0;
  xx[2] = xx[1] * (state[14] * state[15] - state[13] * state[16]);
  xx[3] = xx[2] * xx[2];
  xx[4] = 1.0;
  xx[5] = (state[13] * state[13] + state[14] * state[14]) * xx[1] - xx[4];
  xx[6] = xx[5] * xx[5];
  xx[7] = sqrt(xx[3] + xx[6]);
  xx[8] = xx[7] == 0.0 ? 0.0 : - xx[2] / xx[7];
  xx[9] = xx[6] + xx[3];
  xx[3] = sqrt(xx[9]);
  xx[6] = xx[3] == 0.0 ? 0.0 : xx[5] / xx[3];
  xx[10] = 0.0;
  xx[11] = (state[14] * state[16] + state[13] * state[15]) * xx[1];
  xx[1] = sqrt(xx[9] + xx[11] * xx[11]);
  xx[12] = xx[1] == 0.0 ? 0.0 : xx[5] / xx[1];
  xx[14] = xx[8];
  xx[15] = xx[6];
  xx[16] = xx[10];
  xx[17] = xx[8];
  xx[18] = xx[8];
  xx[19] = xx[12];
  xx[6] = xx[13 + (partialType)];
  xx[8] = xx[7] == 0.0 ? 0.0 : xx[5] / xx[7];
  xx[7] = xx[3] == 0.0 ? 0.0 : xx[2] / xx[3];
  xx[3] = xx[1] == 0.0 ? 0.0 : xx[2] / xx[1];
  xx[13] = xx[8];
  xx[14] = xx[7];
  xx[15] = xx[10];
  xx[16] = xx[8];
  xx[17] = xx[8];
  xx[18] = xx[3];
  xx[2] = xx[12 + (partialType)];
  xx[3] = xx[1] == 0.0 ? 0.0 : xx[11] / xx[1];
  xx[13] = xx[10];
  xx[14] = xx[10];
  xx[15] = xx[4];
  xx[16] = xx[10];
  xx[17] = xx[10];
  xx[18] = xx[3];
  xx[1] = xx[12 + (partialType)];
  xx[3] = xx[11] * xx[5];
  xx[5] = sqrt(xx[9] * xx[9] + xx[3] * xx[3]);
  xx[7] = xx[5] == 0.0 ? 0.0 : xx[9] / xx[5];
  xx[12] = xx[10];
  xx[13] = xx[10];
  xx[14] = xx[10];
  xx[15] = xx[7];
  xx[16] = xx[10];
  xx[17] = xx[10];
  xx[7] = xx[11 + (partialType)];
  xx[12] = xx[10];
  xx[13] = xx[10];
  xx[14] = xx[10];
  xx[15] = xx[10];
  xx[16] = xx[10];
  xx[17] = xx[10];
  xx[8] = xx[11 + (partialType)];
  xx[9] = xx[5] == 0.0 ? 0.0 : xx[3] / xx[5];
  xx[12] = xx[4];
  xx[13] = xx[4];
  xx[14] = xx[10];
  xx[15] = xx[9];
  xx[16] = xx[10];
  xx[17] = xx[10];
  xx[0] = xx[11 + (partialType)];
  matrix[0] = xx[6];
  matrix[1] = xx[2];
  matrix[2] = xx[1];
  matrix[3] = xx[7];
  matrix[4] = xx[8];
  matrix[5] = xx[0];
  matrix[6] = xx[8];
  matrix[7] = xx[8];
  matrix[8] = xx[8];
}

static void computePosDofBlendMatrix_2_0(const double *state, int partialType,
  double *matrix)
{
  double xx[20];
  xx[0] = 0.0;
  xx[1] = 2.0;
  xx[2] = xx[1] * (state[21] * state[22] - state[20] * state[23]);
  xx[3] = xx[2] * xx[2];
  xx[4] = 1.0;
  xx[5] = (state[20] * state[20] + state[21] * state[21]) * xx[1] - xx[4];
  xx[6] = xx[5] * xx[5];
  xx[7] = sqrt(xx[3] + xx[6]);
  xx[8] = xx[7] == 0.0 ? 0.0 : - xx[2] / xx[7];
  xx[9] = xx[6] + xx[3];
  xx[3] = sqrt(xx[9]);
  xx[6] = xx[3] == 0.0 ? 0.0 : xx[5] / xx[3];
  xx[10] = 0.0;
  xx[11] = (state[21] * state[23] + state[20] * state[22]) * xx[1];
  xx[1] = sqrt(xx[9] + xx[11] * xx[11]);
  xx[12] = xx[1] == 0.0 ? 0.0 : xx[5] / xx[1];
  xx[14] = xx[8];
  xx[15] = xx[6];
  xx[16] = xx[10];
  xx[17] = xx[8];
  xx[18] = xx[8];
  xx[19] = xx[12];
  xx[6] = xx[13 + (partialType)];
  xx[8] = xx[7] == 0.0 ? 0.0 : xx[5] / xx[7];
  xx[7] = xx[3] == 0.0 ? 0.0 : xx[2] / xx[3];
  xx[3] = xx[1] == 0.0 ? 0.0 : xx[2] / xx[1];
  xx[13] = xx[8];
  xx[14] = xx[7];
  xx[15] = xx[10];
  xx[16] = xx[8];
  xx[17] = xx[8];
  xx[18] = xx[3];
  xx[2] = xx[12 + (partialType)];
  xx[3] = xx[1] == 0.0 ? 0.0 : xx[11] / xx[1];
  xx[13] = xx[10];
  xx[14] = xx[10];
  xx[15] = xx[4];
  xx[16] = xx[10];
  xx[17] = xx[10];
  xx[18] = xx[3];
  xx[1] = xx[12 + (partialType)];
  xx[3] = xx[11] * xx[5];
  xx[5] = sqrt(xx[9] * xx[9] + xx[3] * xx[3]);
  xx[7] = xx[5] == 0.0 ? 0.0 : xx[9] / xx[5];
  xx[12] = xx[10];
  xx[13] = xx[10];
  xx[14] = xx[10];
  xx[15] = xx[7];
  xx[16] = xx[10];
  xx[17] = xx[10];
  xx[7] = xx[11 + (partialType)];
  xx[12] = xx[10];
  xx[13] = xx[10];
  xx[14] = xx[10];
  xx[15] = xx[10];
  xx[16] = xx[10];
  xx[17] = xx[10];
  xx[8] = xx[11 + (partialType)];
  xx[9] = xx[5] == 0.0 ? 0.0 : xx[3] / xx[5];
  xx[12] = xx[4];
  xx[13] = xx[4];
  xx[14] = xx[10];
  xx[15] = xx[9];
  xx[16] = xx[10];
  xx[17] = xx[10];
  xx[0] = xx[11 + (partialType)];
  matrix[0] = xx[6];
  matrix[1] = xx[2];
  matrix[2] = xx[1];
  matrix[3] = xx[7];
  matrix[4] = xx[8];
  matrix[5] = xx[0];
  matrix[6] = xx[8];
  matrix[7] = xx[8];
  matrix[8] = xx[8];
}

static void computePosDofBlendMatrix_3_0(const double *state, int partialType,
  double *matrix)
{
  double xx[20];
  xx[0] = 0.0;
  xx[1] = 2.0;
  xx[2] = xx[1] * (state[28] * state[29] - state[27] * state[30]);
  xx[3] = xx[2] * xx[2];
  xx[4] = 1.0;
  xx[5] = (state[27] * state[27] + state[28] * state[28]) * xx[1] - xx[4];
  xx[6] = xx[5] * xx[5];
  xx[7] = sqrt(xx[3] + xx[6]);
  xx[8] = xx[7] == 0.0 ? 0.0 : - xx[2] / xx[7];
  xx[9] = xx[6] + xx[3];
  xx[3] = sqrt(xx[9]);
  xx[6] = xx[3] == 0.0 ? 0.0 : xx[5] / xx[3];
  xx[10] = 0.0;
  xx[11] = (state[28] * state[30] + state[27] * state[29]) * xx[1];
  xx[1] = sqrt(xx[9] + xx[11] * xx[11]);
  xx[12] = xx[1] == 0.0 ? 0.0 : xx[5] / xx[1];
  xx[14] = xx[8];
  xx[15] = xx[6];
  xx[16] = xx[10];
  xx[17] = xx[8];
  xx[18] = xx[8];
  xx[19] = xx[12];
  xx[6] = xx[13 + (partialType)];
  xx[8] = xx[7] == 0.0 ? 0.0 : xx[5] / xx[7];
  xx[7] = xx[3] == 0.0 ? 0.0 : xx[2] / xx[3];
  xx[3] = xx[1] == 0.0 ? 0.0 : xx[2] / xx[1];
  xx[13] = xx[8];
  xx[14] = xx[7];
  xx[15] = xx[10];
  xx[16] = xx[8];
  xx[17] = xx[8];
  xx[18] = xx[3];
  xx[2] = xx[12 + (partialType)];
  xx[3] = xx[1] == 0.0 ? 0.0 : xx[11] / xx[1];
  xx[13] = xx[10];
  xx[14] = xx[10];
  xx[15] = xx[4];
  xx[16] = xx[10];
  xx[17] = xx[10];
  xx[18] = xx[3];
  xx[1] = xx[12 + (partialType)];
  xx[3] = xx[11] * xx[5];
  xx[5] = sqrt(xx[9] * xx[9] + xx[3] * xx[3]);
  xx[7] = xx[5] == 0.0 ? 0.0 : xx[9] / xx[5];
  xx[12] = xx[10];
  xx[13] = xx[10];
  xx[14] = xx[10];
  xx[15] = xx[7];
  xx[16] = xx[10];
  xx[17] = xx[10];
  xx[7] = xx[11 + (partialType)];
  xx[12] = xx[10];
  xx[13] = xx[10];
  xx[14] = xx[10];
  xx[15] = xx[10];
  xx[16] = xx[10];
  xx[17] = xx[10];
  xx[8] = xx[11 + (partialType)];
  xx[9] = xx[5] == 0.0 ? 0.0 : xx[3] / xx[5];
  xx[12] = xx[4];
  xx[13] = xx[4];
  xx[14] = xx[10];
  xx[15] = xx[9];
  xx[16] = xx[10];
  xx[17] = xx[10];
  xx[0] = xx[11 + (partialType)];
  matrix[0] = xx[6];
  matrix[1] = xx[2];
  matrix[2] = xx[1];
  matrix[3] = xx[7];
  matrix[4] = xx[8];
  matrix[5] = xx[0];
  matrix[6] = xx[8];
  matrix[7] = xx[8];
  matrix[8] = xx[8];
}

static void computePosDofBlendMatrix_6_0(const double *state, int partialType,
  double *matrix)
{
  double xx[20];
  xx[0] = 0.0;
  xx[1] = 2.0;
  xx[2] = xx[1] * (state[39] * state[40] - state[38] * state[41]);
  xx[3] = xx[2] * xx[2];
  xx[4] = 1.0;
  xx[5] = (state[38] * state[38] + state[39] * state[39]) * xx[1] - xx[4];
  xx[6] = xx[5] * xx[5];
  xx[7] = sqrt(xx[3] + xx[6]);
  xx[8] = xx[7] == 0.0 ? 0.0 : - xx[2] / xx[7];
  xx[9] = xx[6] + xx[3];
  xx[3] = sqrt(xx[9]);
  xx[6] = xx[3] == 0.0 ? 0.0 : xx[5] / xx[3];
  xx[10] = 0.0;
  xx[11] = (state[39] * state[41] + state[38] * state[40]) * xx[1];
  xx[1] = sqrt(xx[9] + xx[11] * xx[11]);
  xx[12] = xx[1] == 0.0 ? 0.0 : xx[5] / xx[1];
  xx[14] = xx[8];
  xx[15] = xx[6];
  xx[16] = xx[10];
  xx[17] = xx[8];
  xx[18] = xx[8];
  xx[19] = xx[12];
  xx[6] = xx[13 + (partialType)];
  xx[8] = xx[7] == 0.0 ? 0.0 : xx[5] / xx[7];
  xx[7] = xx[3] == 0.0 ? 0.0 : xx[2] / xx[3];
  xx[3] = xx[1] == 0.0 ? 0.0 : xx[2] / xx[1];
  xx[13] = xx[8];
  xx[14] = xx[7];
  xx[15] = xx[10];
  xx[16] = xx[8];
  xx[17] = xx[8];
  xx[18] = xx[3];
  xx[2] = xx[12 + (partialType)];
  xx[3] = xx[1] == 0.0 ? 0.0 : xx[11] / xx[1];
  xx[13] = xx[10];
  xx[14] = xx[10];
  xx[15] = xx[4];
  xx[16] = xx[10];
  xx[17] = xx[10];
  xx[18] = xx[3];
  xx[1] = xx[12 + (partialType)];
  xx[3] = xx[11] * xx[5];
  xx[5] = sqrt(xx[9] * xx[9] + xx[3] * xx[3]);
  xx[7] = xx[5] == 0.0 ? 0.0 : xx[9] / xx[5];
  xx[12] = xx[10];
  xx[13] = xx[10];
  xx[14] = xx[10];
  xx[15] = xx[7];
  xx[16] = xx[10];
  xx[17] = xx[10];
  xx[7] = xx[11 + (partialType)];
  xx[12] = xx[10];
  xx[13] = xx[10];
  xx[14] = xx[10];
  xx[15] = xx[10];
  xx[16] = xx[10];
  xx[17] = xx[10];
  xx[8] = xx[11 + (partialType)];
  xx[9] = xx[5] == 0.0 ? 0.0 : xx[3] / xx[5];
  xx[12] = xx[4];
  xx[13] = xx[4];
  xx[14] = xx[10];
  xx[15] = xx[9];
  xx[16] = xx[10];
  xx[17] = xx[10];
  xx[0] = xx[11 + (partialType)];
  matrix[0] = xx[6];
  matrix[1] = xx[2];
  matrix[2] = xx[1];
  matrix[3] = xx[7];
  matrix[4] = xx[8];
  matrix[5] = xx[0];
  matrix[6] = xx[8];
  matrix[7] = xx[8];
  matrix[8] = xx[8];
}

static void computePosDofBlendMatrix_7_0(const double *state, int partialType,
  double *matrix)
{
  double xx[20];
  xx[0] = 0.0;
  xx[1] = 2.0;
  xx[2] = xx[1] * (state[46] * state[47] - state[45] * state[48]);
  xx[3] = xx[2] * xx[2];
  xx[4] = 1.0;
  xx[5] = (state[45] * state[45] + state[46] * state[46]) * xx[1] - xx[4];
  xx[6] = xx[5] * xx[5];
  xx[7] = sqrt(xx[3] + xx[6]);
  xx[8] = xx[7] == 0.0 ? 0.0 : - xx[2] / xx[7];
  xx[9] = xx[6] + xx[3];
  xx[3] = sqrt(xx[9]);
  xx[6] = xx[3] == 0.0 ? 0.0 : xx[5] / xx[3];
  xx[10] = 0.0;
  xx[11] = (state[46] * state[48] + state[45] * state[47]) * xx[1];
  xx[1] = sqrt(xx[9] + xx[11] * xx[11]);
  xx[12] = xx[1] == 0.0 ? 0.0 : xx[5] / xx[1];
  xx[14] = xx[8];
  xx[15] = xx[6];
  xx[16] = xx[10];
  xx[17] = xx[8];
  xx[18] = xx[8];
  xx[19] = xx[12];
  xx[6] = xx[13 + (partialType)];
  xx[8] = xx[7] == 0.0 ? 0.0 : xx[5] / xx[7];
  xx[7] = xx[3] == 0.0 ? 0.0 : xx[2] / xx[3];
  xx[3] = xx[1] == 0.0 ? 0.0 : xx[2] / xx[1];
  xx[13] = xx[8];
  xx[14] = xx[7];
  xx[15] = xx[10];
  xx[16] = xx[8];
  xx[17] = xx[8];
  xx[18] = xx[3];
  xx[2] = xx[12 + (partialType)];
  xx[3] = xx[1] == 0.0 ? 0.0 : xx[11] / xx[1];
  xx[13] = xx[10];
  xx[14] = xx[10];
  xx[15] = xx[4];
  xx[16] = xx[10];
  xx[17] = xx[10];
  xx[18] = xx[3];
  xx[1] = xx[12 + (partialType)];
  xx[3] = xx[11] * xx[5];
  xx[5] = sqrt(xx[9] * xx[9] + xx[3] * xx[3]);
  xx[7] = xx[5] == 0.0 ? 0.0 : xx[9] / xx[5];
  xx[12] = xx[10];
  xx[13] = xx[10];
  xx[14] = xx[10];
  xx[15] = xx[7];
  xx[16] = xx[10];
  xx[17] = xx[10];
  xx[7] = xx[11 + (partialType)];
  xx[12] = xx[10];
  xx[13] = xx[10];
  xx[14] = xx[10];
  xx[15] = xx[10];
  xx[16] = xx[10];
  xx[17] = xx[10];
  xx[8] = xx[11 + (partialType)];
  xx[9] = xx[5] == 0.0 ? 0.0 : xx[3] / xx[5];
  xx[12] = xx[4];
  xx[13] = xx[4];
  xx[14] = xx[10];
  xx[15] = xx[9];
  xx[16] = xx[10];
  xx[17] = xx[10];
  xx[0] = xx[11 + (partialType)];
  matrix[0] = xx[6];
  matrix[1] = xx[2];
  matrix[2] = xx[1];
  matrix[3] = xx[7];
  matrix[4] = xx[8];
  matrix[5] = xx[0];
  matrix[6] = xx[8];
  matrix[7] = xx[8];
  matrix[8] = xx[8];
}

static void computePosDofBlendMatrix_8_0(const double *state, int partialType,
  double *matrix)
{
  double xx[20];
  xx[0] = 0.0;
  xx[1] = 2.0;
  xx[2] = xx[1] * (state[53] * state[54] - state[52] * state[55]);
  xx[3] = xx[2] * xx[2];
  xx[4] = 1.0;
  xx[5] = (state[52] * state[52] + state[53] * state[53]) * xx[1] - xx[4];
  xx[6] = xx[5] * xx[5];
  xx[7] = sqrt(xx[3] + xx[6]);
  xx[8] = xx[7] == 0.0 ? 0.0 : - xx[2] / xx[7];
  xx[9] = xx[6] + xx[3];
  xx[3] = sqrt(xx[9]);
  xx[6] = xx[3] == 0.0 ? 0.0 : xx[5] / xx[3];
  xx[10] = 0.0;
  xx[11] = (state[53] * state[55] + state[52] * state[54]) * xx[1];
  xx[1] = sqrt(xx[9] + xx[11] * xx[11]);
  xx[12] = xx[1] == 0.0 ? 0.0 : xx[5] / xx[1];
  xx[14] = xx[8];
  xx[15] = xx[6];
  xx[16] = xx[10];
  xx[17] = xx[8];
  xx[18] = xx[8];
  xx[19] = xx[12];
  xx[6] = xx[13 + (partialType)];
  xx[8] = xx[7] == 0.0 ? 0.0 : xx[5] / xx[7];
  xx[7] = xx[3] == 0.0 ? 0.0 : xx[2] / xx[3];
  xx[3] = xx[1] == 0.0 ? 0.0 : xx[2] / xx[1];
  xx[13] = xx[8];
  xx[14] = xx[7];
  xx[15] = xx[10];
  xx[16] = xx[8];
  xx[17] = xx[8];
  xx[18] = xx[3];
  xx[2] = xx[12 + (partialType)];
  xx[3] = xx[1] == 0.0 ? 0.0 : xx[11] / xx[1];
  xx[13] = xx[10];
  xx[14] = xx[10];
  xx[15] = xx[4];
  xx[16] = xx[10];
  xx[17] = xx[10];
  xx[18] = xx[3];
  xx[1] = xx[12 + (partialType)];
  xx[3] = xx[11] * xx[5];
  xx[5] = sqrt(xx[9] * xx[9] + xx[3] * xx[3]);
  xx[7] = xx[5] == 0.0 ? 0.0 : xx[9] / xx[5];
  xx[12] = xx[10];
  xx[13] = xx[10];
  xx[14] = xx[10];
  xx[15] = xx[7];
  xx[16] = xx[10];
  xx[17] = xx[10];
  xx[7] = xx[11 + (partialType)];
  xx[12] = xx[10];
  xx[13] = xx[10];
  xx[14] = xx[10];
  xx[15] = xx[10];
  xx[16] = xx[10];
  xx[17] = xx[10];
  xx[8] = xx[11 + (partialType)];
  xx[9] = xx[5] == 0.0 ? 0.0 : xx[3] / xx[5];
  xx[12] = xx[4];
  xx[13] = xx[4];
  xx[14] = xx[10];
  xx[15] = xx[9];
  xx[16] = xx[10];
  xx[17] = xx[10];
  xx[0] = xx[11 + (partialType)];
  matrix[0] = xx[6];
  matrix[1] = xx[2];
  matrix[2] = xx[1];
  matrix[3] = xx[7];
  matrix[4] = xx[8];
  matrix[5] = xx[0];
  matrix[6] = xx[8];
  matrix[7] = xx[8];
  matrix[8] = xx[8];
}

static void computePosDofBlendMatrix_9_0(const double *state, int partialType,
  double *matrix)
{
  double xx[20];
  xx[0] = 0.0;
  xx[1] = 2.0;
  xx[2] = xx[1] * (state[60] * state[61] - state[59] * state[62]);
  xx[3] = xx[2] * xx[2];
  xx[4] = 1.0;
  xx[5] = (state[59] * state[59] + state[60] * state[60]) * xx[1] - xx[4];
  xx[6] = xx[5] * xx[5];
  xx[7] = sqrt(xx[3] + xx[6]);
  xx[8] = xx[7] == 0.0 ? 0.0 : - xx[2] / xx[7];
  xx[9] = xx[6] + xx[3];
  xx[3] = sqrt(xx[9]);
  xx[6] = xx[3] == 0.0 ? 0.0 : xx[5] / xx[3];
  xx[10] = 0.0;
  xx[11] = (state[60] * state[62] + state[59] * state[61]) * xx[1];
  xx[1] = sqrt(xx[9] + xx[11] * xx[11]);
  xx[12] = xx[1] == 0.0 ? 0.0 : xx[5] / xx[1];
  xx[14] = xx[8];
  xx[15] = xx[6];
  xx[16] = xx[10];
  xx[17] = xx[8];
  xx[18] = xx[8];
  xx[19] = xx[12];
  xx[6] = xx[13 + (partialType)];
  xx[8] = xx[7] == 0.0 ? 0.0 : xx[5] / xx[7];
  xx[7] = xx[3] == 0.0 ? 0.0 : xx[2] / xx[3];
  xx[3] = xx[1] == 0.0 ? 0.0 : xx[2] / xx[1];
  xx[13] = xx[8];
  xx[14] = xx[7];
  xx[15] = xx[10];
  xx[16] = xx[8];
  xx[17] = xx[8];
  xx[18] = xx[3];
  xx[2] = xx[12 + (partialType)];
  xx[3] = xx[1] == 0.0 ? 0.0 : xx[11] / xx[1];
  xx[13] = xx[10];
  xx[14] = xx[10];
  xx[15] = xx[4];
  xx[16] = xx[10];
  xx[17] = xx[10];
  xx[18] = xx[3];
  xx[1] = xx[12 + (partialType)];
  xx[3] = xx[11] * xx[5];
  xx[5] = sqrt(xx[9] * xx[9] + xx[3] * xx[3]);
  xx[7] = xx[5] == 0.0 ? 0.0 : xx[9] / xx[5];
  xx[12] = xx[10];
  xx[13] = xx[10];
  xx[14] = xx[10];
  xx[15] = xx[7];
  xx[16] = xx[10];
  xx[17] = xx[10];
  xx[7] = xx[11 + (partialType)];
  xx[12] = xx[10];
  xx[13] = xx[10];
  xx[14] = xx[10];
  xx[15] = xx[10];
  xx[16] = xx[10];
  xx[17] = xx[10];
  xx[8] = xx[11 + (partialType)];
  xx[9] = xx[5] == 0.0 ? 0.0 : xx[3] / xx[5];
  xx[12] = xx[4];
  xx[13] = xx[4];
  xx[14] = xx[10];
  xx[15] = xx[9];
  xx[16] = xx[10];
  xx[17] = xx[10];
  xx[0] = xx[11 + (partialType)];
  matrix[0] = xx[6];
  matrix[1] = xx[2];
  matrix[2] = xx[1];
  matrix[3] = xx[7];
  matrix[4] = xx[8];
  matrix[5] = xx[0];
  matrix[6] = xx[8];
  matrix[7] = xx[8];
  matrix[8] = xx[8];
}

static void computePosDofBlendMatrix_10_0(const double *state, int partialType,
  double *matrix)
{
  double xx[20];
  xx[0] = 0.0;
  xx[1] = 2.0;
  xx[2] = xx[1] * (state[67] * state[68] - state[66] * state[69]);
  xx[3] = xx[2] * xx[2];
  xx[4] = 1.0;
  xx[5] = (state[66] * state[66] + state[67] * state[67]) * xx[1] - xx[4];
  xx[6] = xx[5] * xx[5];
  xx[7] = sqrt(xx[3] + xx[6]);
  xx[8] = xx[7] == 0.0 ? 0.0 : - xx[2] / xx[7];
  xx[9] = xx[6] + xx[3];
  xx[3] = sqrt(xx[9]);
  xx[6] = xx[3] == 0.0 ? 0.0 : xx[5] / xx[3];
  xx[10] = 0.0;
  xx[11] = (state[67] * state[69] + state[66] * state[68]) * xx[1];
  xx[1] = sqrt(xx[9] + xx[11] * xx[11]);
  xx[12] = xx[1] == 0.0 ? 0.0 : xx[5] / xx[1];
  xx[14] = xx[8];
  xx[15] = xx[6];
  xx[16] = xx[10];
  xx[17] = xx[8];
  xx[18] = xx[8];
  xx[19] = xx[12];
  xx[6] = xx[13 + (partialType)];
  xx[8] = xx[7] == 0.0 ? 0.0 : xx[5] / xx[7];
  xx[7] = xx[3] == 0.0 ? 0.0 : xx[2] / xx[3];
  xx[3] = xx[1] == 0.0 ? 0.0 : xx[2] / xx[1];
  xx[13] = xx[8];
  xx[14] = xx[7];
  xx[15] = xx[10];
  xx[16] = xx[8];
  xx[17] = xx[8];
  xx[18] = xx[3];
  xx[2] = xx[12 + (partialType)];
  xx[3] = xx[1] == 0.0 ? 0.0 : xx[11] / xx[1];
  xx[13] = xx[10];
  xx[14] = xx[10];
  xx[15] = xx[4];
  xx[16] = xx[10];
  xx[17] = xx[10];
  xx[18] = xx[3];
  xx[1] = xx[12 + (partialType)];
  xx[3] = xx[11] * xx[5];
  xx[5] = sqrt(xx[9] * xx[9] + xx[3] * xx[3]);
  xx[7] = xx[5] == 0.0 ? 0.0 : xx[9] / xx[5];
  xx[12] = xx[10];
  xx[13] = xx[10];
  xx[14] = xx[10];
  xx[15] = xx[7];
  xx[16] = xx[10];
  xx[17] = xx[10];
  xx[7] = xx[11 + (partialType)];
  xx[12] = xx[10];
  xx[13] = xx[10];
  xx[14] = xx[10];
  xx[15] = xx[10];
  xx[16] = xx[10];
  xx[17] = xx[10];
  xx[8] = xx[11 + (partialType)];
  xx[9] = xx[5] == 0.0 ? 0.0 : xx[3] / xx[5];
  xx[12] = xx[4];
  xx[13] = xx[4];
  xx[14] = xx[10];
  xx[15] = xx[9];
  xx[16] = xx[10];
  xx[17] = xx[10];
  xx[0] = xx[11 + (partialType)];
  matrix[0] = xx[6];
  matrix[1] = xx[2];
  matrix[2] = xx[1];
  matrix[3] = xx[7];
  matrix[4] = xx[8];
  matrix[5] = xx[0];
  matrix[6] = xx[8];
  matrix[7] = xx[8];
  matrix[8] = xx[8];
}

static void computePosDofBlendMatrix_11_0(const double *state, int partialType,
  double *matrix)
{
  double xx[20];
  xx[0] = 0.0;
  xx[1] = 2.0;
  xx[2] = xx[1] * (state[74] * state[75] - state[73] * state[76]);
  xx[3] = xx[2] * xx[2];
  xx[4] = 1.0;
  xx[5] = (state[73] * state[73] + state[74] * state[74]) * xx[1] - xx[4];
  xx[6] = xx[5] * xx[5];
  xx[7] = sqrt(xx[3] + xx[6]);
  xx[8] = xx[7] == 0.0 ? 0.0 : - xx[2] / xx[7];
  xx[9] = xx[6] + xx[3];
  xx[3] = sqrt(xx[9]);
  xx[6] = xx[3] == 0.0 ? 0.0 : xx[5] / xx[3];
  xx[10] = 0.0;
  xx[11] = (state[74] * state[76] + state[73] * state[75]) * xx[1];
  xx[1] = sqrt(xx[9] + xx[11] * xx[11]);
  xx[12] = xx[1] == 0.0 ? 0.0 : xx[5] / xx[1];
  xx[14] = xx[8];
  xx[15] = xx[6];
  xx[16] = xx[10];
  xx[17] = xx[8];
  xx[18] = xx[8];
  xx[19] = xx[12];
  xx[6] = xx[13 + (partialType)];
  xx[8] = xx[7] == 0.0 ? 0.0 : xx[5] / xx[7];
  xx[7] = xx[3] == 0.0 ? 0.0 : xx[2] / xx[3];
  xx[3] = xx[1] == 0.0 ? 0.0 : xx[2] / xx[1];
  xx[13] = xx[8];
  xx[14] = xx[7];
  xx[15] = xx[10];
  xx[16] = xx[8];
  xx[17] = xx[8];
  xx[18] = xx[3];
  xx[2] = xx[12 + (partialType)];
  xx[3] = xx[1] == 0.0 ? 0.0 : xx[11] / xx[1];
  xx[13] = xx[10];
  xx[14] = xx[10];
  xx[15] = xx[4];
  xx[16] = xx[10];
  xx[17] = xx[10];
  xx[18] = xx[3];
  xx[1] = xx[12 + (partialType)];
  xx[3] = xx[11] * xx[5];
  xx[5] = sqrt(xx[9] * xx[9] + xx[3] * xx[3]);
  xx[7] = xx[5] == 0.0 ? 0.0 : xx[9] / xx[5];
  xx[12] = xx[10];
  xx[13] = xx[10];
  xx[14] = xx[10];
  xx[15] = xx[7];
  xx[16] = xx[10];
  xx[17] = xx[10];
  xx[7] = xx[11 + (partialType)];
  xx[12] = xx[10];
  xx[13] = xx[10];
  xx[14] = xx[10];
  xx[15] = xx[10];
  xx[16] = xx[10];
  xx[17] = xx[10];
  xx[8] = xx[11 + (partialType)];
  xx[9] = xx[5] == 0.0 ? 0.0 : xx[3] / xx[5];
  xx[12] = xx[4];
  xx[13] = xx[4];
  xx[14] = xx[10];
  xx[15] = xx[9];
  xx[16] = xx[10];
  xx[17] = xx[10];
  xx[0] = xx[11 + (partialType)];
  matrix[0] = xx[6];
  matrix[1] = xx[2];
  matrix[2] = xx[1];
  matrix[3] = xx[7];
  matrix[4] = xx[8];
  matrix[5] = xx[0];
  matrix[6] = xx[8];
  matrix[7] = xx[8];
  matrix[8] = xx[8];
}

static void computePosDofBlendMatrix_12_0(const double *state, int partialType,
  double *matrix)
{
  double xx[20];
  xx[0] = 0.0;
  xx[1] = 2.0;
  xx[2] = xx[1] * (state[81] * state[82] - state[80] * state[83]);
  xx[3] = xx[2] * xx[2];
  xx[4] = 1.0;
  xx[5] = (state[80] * state[80] + state[81] * state[81]) * xx[1] - xx[4];
  xx[6] = xx[5] * xx[5];
  xx[7] = sqrt(xx[3] + xx[6]);
  xx[8] = xx[7] == 0.0 ? 0.0 : - xx[2] / xx[7];
  xx[9] = xx[6] + xx[3];
  xx[3] = sqrt(xx[9]);
  xx[6] = xx[3] == 0.0 ? 0.0 : xx[5] / xx[3];
  xx[10] = 0.0;
  xx[11] = (state[81] * state[83] + state[80] * state[82]) * xx[1];
  xx[1] = sqrt(xx[9] + xx[11] * xx[11]);
  xx[12] = xx[1] == 0.0 ? 0.0 : xx[5] / xx[1];
  xx[14] = xx[8];
  xx[15] = xx[6];
  xx[16] = xx[10];
  xx[17] = xx[8];
  xx[18] = xx[8];
  xx[19] = xx[12];
  xx[6] = xx[13 + (partialType)];
  xx[8] = xx[7] == 0.0 ? 0.0 : xx[5] / xx[7];
  xx[7] = xx[3] == 0.0 ? 0.0 : xx[2] / xx[3];
  xx[3] = xx[1] == 0.0 ? 0.0 : xx[2] / xx[1];
  xx[13] = xx[8];
  xx[14] = xx[7];
  xx[15] = xx[10];
  xx[16] = xx[8];
  xx[17] = xx[8];
  xx[18] = xx[3];
  xx[2] = xx[12 + (partialType)];
  xx[3] = xx[1] == 0.0 ? 0.0 : xx[11] / xx[1];
  xx[13] = xx[10];
  xx[14] = xx[10];
  xx[15] = xx[4];
  xx[16] = xx[10];
  xx[17] = xx[10];
  xx[18] = xx[3];
  xx[1] = xx[12 + (partialType)];
  xx[3] = xx[11] * xx[5];
  xx[5] = sqrt(xx[9] * xx[9] + xx[3] * xx[3]);
  xx[7] = xx[5] == 0.0 ? 0.0 : xx[9] / xx[5];
  xx[12] = xx[10];
  xx[13] = xx[10];
  xx[14] = xx[10];
  xx[15] = xx[7];
  xx[16] = xx[10];
  xx[17] = xx[10];
  xx[7] = xx[11 + (partialType)];
  xx[12] = xx[10];
  xx[13] = xx[10];
  xx[14] = xx[10];
  xx[15] = xx[10];
  xx[16] = xx[10];
  xx[17] = xx[10];
  xx[8] = xx[11 + (partialType)];
  xx[9] = xx[5] == 0.0 ? 0.0 : xx[3] / xx[5];
  xx[12] = xx[4];
  xx[13] = xx[4];
  xx[14] = xx[10];
  xx[15] = xx[9];
  xx[16] = xx[10];
  xx[17] = xx[10];
  xx[0] = xx[11 + (partialType)];
  matrix[0] = xx[6];
  matrix[1] = xx[2];
  matrix[2] = xx[1];
  matrix[3] = xx[7];
  matrix[4] = xx[8];
  matrix[5] = xx[0];
  matrix[6] = xx[8];
  matrix[7] = xx[8];
  matrix[8] = xx[8];
}

static void computePosDofBlendMatrix_13_0(const double *state, int partialType,
  double *matrix)
{
  double xx[20];
  xx[0] = 0.0;
  xx[1] = 2.0;
  xx[2] = xx[1] * (state[88] * state[89] - state[87] * state[90]);
  xx[3] = xx[2] * xx[2];
  xx[4] = 1.0;
  xx[5] = (state[87] * state[87] + state[88] * state[88]) * xx[1] - xx[4];
  xx[6] = xx[5] * xx[5];
  xx[7] = sqrt(xx[3] + xx[6]);
  xx[8] = xx[7] == 0.0 ? 0.0 : - xx[2] / xx[7];
  xx[9] = xx[6] + xx[3];
  xx[3] = sqrt(xx[9]);
  xx[6] = xx[3] == 0.0 ? 0.0 : xx[5] / xx[3];
  xx[10] = 0.0;
  xx[11] = (state[88] * state[90] + state[87] * state[89]) * xx[1];
  xx[1] = sqrt(xx[9] + xx[11] * xx[11]);
  xx[12] = xx[1] == 0.0 ? 0.0 : xx[5] / xx[1];
  xx[14] = xx[8];
  xx[15] = xx[6];
  xx[16] = xx[10];
  xx[17] = xx[8];
  xx[18] = xx[8];
  xx[19] = xx[12];
  xx[6] = xx[13 + (partialType)];
  xx[8] = xx[7] == 0.0 ? 0.0 : xx[5] / xx[7];
  xx[7] = xx[3] == 0.0 ? 0.0 : xx[2] / xx[3];
  xx[3] = xx[1] == 0.0 ? 0.0 : xx[2] / xx[1];
  xx[13] = xx[8];
  xx[14] = xx[7];
  xx[15] = xx[10];
  xx[16] = xx[8];
  xx[17] = xx[8];
  xx[18] = xx[3];
  xx[2] = xx[12 + (partialType)];
  xx[3] = xx[1] == 0.0 ? 0.0 : xx[11] / xx[1];
  xx[13] = xx[10];
  xx[14] = xx[10];
  xx[15] = xx[4];
  xx[16] = xx[10];
  xx[17] = xx[10];
  xx[18] = xx[3];
  xx[1] = xx[12 + (partialType)];
  xx[3] = xx[11] * xx[5];
  xx[5] = sqrt(xx[9] * xx[9] + xx[3] * xx[3]);
  xx[7] = xx[5] == 0.0 ? 0.0 : xx[9] / xx[5];
  xx[12] = xx[10];
  xx[13] = xx[10];
  xx[14] = xx[10];
  xx[15] = xx[7];
  xx[16] = xx[10];
  xx[17] = xx[10];
  xx[7] = xx[11 + (partialType)];
  xx[12] = xx[10];
  xx[13] = xx[10];
  xx[14] = xx[10];
  xx[15] = xx[10];
  xx[16] = xx[10];
  xx[17] = xx[10];
  xx[8] = xx[11 + (partialType)];
  xx[9] = xx[5] == 0.0 ? 0.0 : xx[3] / xx[5];
  xx[12] = xx[4];
  xx[13] = xx[4];
  xx[14] = xx[10];
  xx[15] = xx[9];
  xx[16] = xx[10];
  xx[17] = xx[10];
  xx[0] = xx[11 + (partialType)];
  matrix[0] = xx[6];
  matrix[1] = xx[2];
  matrix[2] = xx[1];
  matrix[3] = xx[7];
  matrix[4] = xx[8];
  matrix[5] = xx[0];
  matrix[6] = xx[8];
  matrix[7] = xx[8];
  matrix[8] = xx[8];
}

static void computePosDofBlendMatrix_16_0(const double *state, int partialType,
  double *matrix)
{
  double xx[20];
  xx[0] = 0.0;
  xx[1] = 2.0;
  xx[2] = xx[1] * (state[99] * state[100] - state[98] * state[101]);
  xx[3] = xx[2] * xx[2];
  xx[4] = 1.0;
  xx[5] = (state[98] * state[98] + state[99] * state[99]) * xx[1] - xx[4];
  xx[6] = xx[5] * xx[5];
  xx[7] = sqrt(xx[3] + xx[6]);
  xx[8] = xx[7] == 0.0 ? 0.0 : - xx[2] / xx[7];
  xx[9] = xx[6] + xx[3];
  xx[3] = sqrt(xx[9]);
  xx[6] = xx[3] == 0.0 ? 0.0 : xx[5] / xx[3];
  xx[10] = 0.0;
  xx[11] = (state[99] * state[101] + state[98] * state[100]) * xx[1];
  xx[1] = sqrt(xx[9] + xx[11] * xx[11]);
  xx[12] = xx[1] == 0.0 ? 0.0 : xx[5] / xx[1];
  xx[14] = xx[8];
  xx[15] = xx[6];
  xx[16] = xx[10];
  xx[17] = xx[8];
  xx[18] = xx[8];
  xx[19] = xx[12];
  xx[6] = xx[13 + (partialType)];
  xx[8] = xx[7] == 0.0 ? 0.0 : xx[5] / xx[7];
  xx[7] = xx[3] == 0.0 ? 0.0 : xx[2] / xx[3];
  xx[3] = xx[1] == 0.0 ? 0.0 : xx[2] / xx[1];
  xx[13] = xx[8];
  xx[14] = xx[7];
  xx[15] = xx[10];
  xx[16] = xx[8];
  xx[17] = xx[8];
  xx[18] = xx[3];
  xx[2] = xx[12 + (partialType)];
  xx[3] = xx[1] == 0.0 ? 0.0 : xx[11] / xx[1];
  xx[13] = xx[10];
  xx[14] = xx[10];
  xx[15] = xx[4];
  xx[16] = xx[10];
  xx[17] = xx[10];
  xx[18] = xx[3];
  xx[1] = xx[12 + (partialType)];
  xx[3] = xx[11] * xx[5];
  xx[5] = sqrt(xx[9] * xx[9] + xx[3] * xx[3]);
  xx[7] = xx[5] == 0.0 ? 0.0 : xx[9] / xx[5];
  xx[12] = xx[10];
  xx[13] = xx[10];
  xx[14] = xx[10];
  xx[15] = xx[7];
  xx[16] = xx[10];
  xx[17] = xx[10];
  xx[7] = xx[11 + (partialType)];
  xx[12] = xx[10];
  xx[13] = xx[10];
  xx[14] = xx[10];
  xx[15] = xx[10];
  xx[16] = xx[10];
  xx[17] = xx[10];
  xx[8] = xx[11 + (partialType)];
  xx[9] = xx[5] == 0.0 ? 0.0 : xx[3] / xx[5];
  xx[12] = xx[4];
  xx[13] = xx[4];
  xx[14] = xx[10];
  xx[15] = xx[9];
  xx[16] = xx[10];
  xx[17] = xx[10];
  xx[0] = xx[11 + (partialType)];
  matrix[0] = xx[6];
  matrix[1] = xx[2];
  matrix[2] = xx[1];
  matrix[3] = xx[7];
  matrix[4] = xx[8];
  matrix[5] = xx[0];
  matrix[6] = xx[8];
  matrix[7] = xx[8];
  matrix[8] = xx[8];
}

static void computePosDofBlendMatrix_17_0(const double *state, int partialType,
  double *matrix)
{
  double xx[20];
  xx[0] = 0.0;
  xx[1] = 2.0;
  xx[2] = xx[1] * (state[106] * state[107] - state[105] * state[108]);
  xx[3] = xx[2] * xx[2];
  xx[4] = 1.0;
  xx[5] = (state[105] * state[105] + state[106] * state[106]) * xx[1] - xx[4];
  xx[6] = xx[5] * xx[5];
  xx[7] = sqrt(xx[3] + xx[6]);
  xx[8] = xx[7] == 0.0 ? 0.0 : - xx[2] / xx[7];
  xx[9] = xx[6] + xx[3];
  xx[3] = sqrt(xx[9]);
  xx[6] = xx[3] == 0.0 ? 0.0 : xx[5] / xx[3];
  xx[10] = 0.0;
  xx[11] = (state[106] * state[108] + state[105] * state[107]) * xx[1];
  xx[1] = sqrt(xx[9] + xx[11] * xx[11]);
  xx[12] = xx[1] == 0.0 ? 0.0 : xx[5] / xx[1];
  xx[14] = xx[8];
  xx[15] = xx[6];
  xx[16] = xx[10];
  xx[17] = xx[8];
  xx[18] = xx[8];
  xx[19] = xx[12];
  xx[6] = xx[13 + (partialType)];
  xx[8] = xx[7] == 0.0 ? 0.0 : xx[5] / xx[7];
  xx[7] = xx[3] == 0.0 ? 0.0 : xx[2] / xx[3];
  xx[3] = xx[1] == 0.0 ? 0.0 : xx[2] / xx[1];
  xx[13] = xx[8];
  xx[14] = xx[7];
  xx[15] = xx[10];
  xx[16] = xx[8];
  xx[17] = xx[8];
  xx[18] = xx[3];
  xx[2] = xx[12 + (partialType)];
  xx[3] = xx[1] == 0.0 ? 0.0 : xx[11] / xx[1];
  xx[13] = xx[10];
  xx[14] = xx[10];
  xx[15] = xx[4];
  xx[16] = xx[10];
  xx[17] = xx[10];
  xx[18] = xx[3];
  xx[1] = xx[12 + (partialType)];
  xx[3] = xx[11] * xx[5];
  xx[5] = sqrt(xx[9] * xx[9] + xx[3] * xx[3]);
  xx[7] = xx[5] == 0.0 ? 0.0 : xx[9] / xx[5];
  xx[12] = xx[10];
  xx[13] = xx[10];
  xx[14] = xx[10];
  xx[15] = xx[7];
  xx[16] = xx[10];
  xx[17] = xx[10];
  xx[7] = xx[11 + (partialType)];
  xx[12] = xx[10];
  xx[13] = xx[10];
  xx[14] = xx[10];
  xx[15] = xx[10];
  xx[16] = xx[10];
  xx[17] = xx[10];
  xx[8] = xx[11 + (partialType)];
  xx[9] = xx[5] == 0.0 ? 0.0 : xx[3] / xx[5];
  xx[12] = xx[4];
  xx[13] = xx[4];
  xx[14] = xx[10];
  xx[15] = xx[9];
  xx[16] = xx[10];
  xx[17] = xx[10];
  xx[0] = xx[11 + (partialType)];
  matrix[0] = xx[6];
  matrix[1] = xx[2];
  matrix[2] = xx[1];
  matrix[3] = xx[7];
  matrix[4] = xx[8];
  matrix[5] = xx[0];
  matrix[6] = xx[8];
  matrix[7] = xx[8];
  matrix[8] = xx[8];
}

static void computePosDofBlendMatrix_18_0(const double *state, int partialType,
  double *matrix)
{
  double xx[20];
  xx[0] = 0.0;
  xx[1] = 2.0;
  xx[2] = xx[1] * (state[113] * state[114] - state[112] * state[115]);
  xx[3] = xx[2] * xx[2];
  xx[4] = 1.0;
  xx[5] = (state[112] * state[112] + state[113] * state[113]) * xx[1] - xx[4];
  xx[6] = xx[5] * xx[5];
  xx[7] = sqrt(xx[3] + xx[6]);
  xx[8] = xx[7] == 0.0 ? 0.0 : - xx[2] / xx[7];
  xx[9] = xx[6] + xx[3];
  xx[3] = sqrt(xx[9]);
  xx[6] = xx[3] == 0.0 ? 0.0 : xx[5] / xx[3];
  xx[10] = 0.0;
  xx[11] = (state[113] * state[115] + state[112] * state[114]) * xx[1];
  xx[1] = sqrt(xx[9] + xx[11] * xx[11]);
  xx[12] = xx[1] == 0.0 ? 0.0 : xx[5] / xx[1];
  xx[14] = xx[8];
  xx[15] = xx[6];
  xx[16] = xx[10];
  xx[17] = xx[8];
  xx[18] = xx[8];
  xx[19] = xx[12];
  xx[6] = xx[13 + (partialType)];
  xx[8] = xx[7] == 0.0 ? 0.0 : xx[5] / xx[7];
  xx[7] = xx[3] == 0.0 ? 0.0 : xx[2] / xx[3];
  xx[3] = xx[1] == 0.0 ? 0.0 : xx[2] / xx[1];
  xx[13] = xx[8];
  xx[14] = xx[7];
  xx[15] = xx[10];
  xx[16] = xx[8];
  xx[17] = xx[8];
  xx[18] = xx[3];
  xx[2] = xx[12 + (partialType)];
  xx[3] = xx[1] == 0.0 ? 0.0 : xx[11] / xx[1];
  xx[13] = xx[10];
  xx[14] = xx[10];
  xx[15] = xx[4];
  xx[16] = xx[10];
  xx[17] = xx[10];
  xx[18] = xx[3];
  xx[1] = xx[12 + (partialType)];
  xx[3] = xx[11] * xx[5];
  xx[5] = sqrt(xx[9] * xx[9] + xx[3] * xx[3]);
  xx[7] = xx[5] == 0.0 ? 0.0 : xx[9] / xx[5];
  xx[12] = xx[10];
  xx[13] = xx[10];
  xx[14] = xx[10];
  xx[15] = xx[7];
  xx[16] = xx[10];
  xx[17] = xx[10];
  xx[7] = xx[11 + (partialType)];
  xx[12] = xx[10];
  xx[13] = xx[10];
  xx[14] = xx[10];
  xx[15] = xx[10];
  xx[16] = xx[10];
  xx[17] = xx[10];
  xx[8] = xx[11 + (partialType)];
  xx[9] = xx[5] == 0.0 ? 0.0 : xx[3] / xx[5];
  xx[12] = xx[4];
  xx[13] = xx[4];
  xx[14] = xx[10];
  xx[15] = xx[9];
  xx[16] = xx[10];
  xx[17] = xx[10];
  xx[0] = xx[11 + (partialType)];
  matrix[0] = xx[6];
  matrix[1] = xx[2];
  matrix[2] = xx[1];
  matrix[3] = xx[7];
  matrix[4] = xx[8];
  matrix[5] = xx[0];
  matrix[6] = xx[8];
  matrix[7] = xx[8];
  matrix[8] = xx[8];
}

static void computePosDofBlendMatrix_19_0(const double *state, int partialType,
  double *matrix)
{
  double xx[20];
  xx[0] = 0.0;
  xx[1] = 2.0;
  xx[2] = xx[1] * (state[120] * state[121] - state[119] * state[122]);
  xx[3] = xx[2] * xx[2];
  xx[4] = 1.0;
  xx[5] = (state[119] * state[119] + state[120] * state[120]) * xx[1] - xx[4];
  xx[6] = xx[5] * xx[5];
  xx[7] = sqrt(xx[3] + xx[6]);
  xx[8] = xx[7] == 0.0 ? 0.0 : - xx[2] / xx[7];
  xx[9] = xx[6] + xx[3];
  xx[3] = sqrt(xx[9]);
  xx[6] = xx[3] == 0.0 ? 0.0 : xx[5] / xx[3];
  xx[10] = 0.0;
  xx[11] = (state[120] * state[122] + state[119] * state[121]) * xx[1];
  xx[1] = sqrt(xx[9] + xx[11] * xx[11]);
  xx[12] = xx[1] == 0.0 ? 0.0 : xx[5] / xx[1];
  xx[14] = xx[8];
  xx[15] = xx[6];
  xx[16] = xx[10];
  xx[17] = xx[8];
  xx[18] = xx[8];
  xx[19] = xx[12];
  xx[6] = xx[13 + (partialType)];
  xx[8] = xx[7] == 0.0 ? 0.0 : xx[5] / xx[7];
  xx[7] = xx[3] == 0.0 ? 0.0 : xx[2] / xx[3];
  xx[3] = xx[1] == 0.0 ? 0.0 : xx[2] / xx[1];
  xx[13] = xx[8];
  xx[14] = xx[7];
  xx[15] = xx[10];
  xx[16] = xx[8];
  xx[17] = xx[8];
  xx[18] = xx[3];
  xx[2] = xx[12 + (partialType)];
  xx[3] = xx[1] == 0.0 ? 0.0 : xx[11] / xx[1];
  xx[13] = xx[10];
  xx[14] = xx[10];
  xx[15] = xx[4];
  xx[16] = xx[10];
  xx[17] = xx[10];
  xx[18] = xx[3];
  xx[1] = xx[12 + (partialType)];
  xx[3] = xx[11] * xx[5];
  xx[5] = sqrt(xx[9] * xx[9] + xx[3] * xx[3]);
  xx[7] = xx[5] == 0.0 ? 0.0 : xx[9] / xx[5];
  xx[12] = xx[10];
  xx[13] = xx[10];
  xx[14] = xx[10];
  xx[15] = xx[7];
  xx[16] = xx[10];
  xx[17] = xx[10];
  xx[7] = xx[11 + (partialType)];
  xx[12] = xx[10];
  xx[13] = xx[10];
  xx[14] = xx[10];
  xx[15] = xx[10];
  xx[16] = xx[10];
  xx[17] = xx[10];
  xx[8] = xx[11 + (partialType)];
  xx[9] = xx[5] == 0.0 ? 0.0 : xx[3] / xx[5];
  xx[12] = xx[4];
  xx[13] = xx[4];
  xx[14] = xx[10];
  xx[15] = xx[9];
  xx[16] = xx[10];
  xx[17] = xx[10];
  xx[0] = xx[11 + (partialType)];
  matrix[0] = xx[6];
  matrix[1] = xx[2];
  matrix[2] = xx[1];
  matrix[3] = xx[7];
  matrix[4] = xx[8];
  matrix[5] = xx[0];
  matrix[6] = xx[8];
  matrix[7] = xx[8];
  matrix[8] = xx[8];
}

static void computePosDofBlendMatrix_20_0(const double *state, int partialType,
  double *matrix)
{
  double xx[20];
  xx[0] = 0.0;
  xx[1] = 2.0;
  xx[2] = xx[1] * (state[127] * state[128] - state[126] * state[129]);
  xx[3] = xx[2] * xx[2];
  xx[4] = 1.0;
  xx[5] = (state[126] * state[126] + state[127] * state[127]) * xx[1] - xx[4];
  xx[6] = xx[5] * xx[5];
  xx[7] = sqrt(xx[3] + xx[6]);
  xx[8] = xx[7] == 0.0 ? 0.0 : - xx[2] / xx[7];
  xx[9] = xx[6] + xx[3];
  xx[3] = sqrt(xx[9]);
  xx[6] = xx[3] == 0.0 ? 0.0 : xx[5] / xx[3];
  xx[10] = 0.0;
  xx[11] = (state[127] * state[129] + state[126] * state[128]) * xx[1];
  xx[1] = sqrt(xx[9] + xx[11] * xx[11]);
  xx[12] = xx[1] == 0.0 ? 0.0 : xx[5] / xx[1];
  xx[14] = xx[8];
  xx[15] = xx[6];
  xx[16] = xx[10];
  xx[17] = xx[8];
  xx[18] = xx[8];
  xx[19] = xx[12];
  xx[6] = xx[13 + (partialType)];
  xx[8] = xx[7] == 0.0 ? 0.0 : xx[5] / xx[7];
  xx[7] = xx[3] == 0.0 ? 0.0 : xx[2] / xx[3];
  xx[3] = xx[1] == 0.0 ? 0.0 : xx[2] / xx[1];
  xx[13] = xx[8];
  xx[14] = xx[7];
  xx[15] = xx[10];
  xx[16] = xx[8];
  xx[17] = xx[8];
  xx[18] = xx[3];
  xx[2] = xx[12 + (partialType)];
  xx[3] = xx[1] == 0.0 ? 0.0 : xx[11] / xx[1];
  xx[13] = xx[10];
  xx[14] = xx[10];
  xx[15] = xx[4];
  xx[16] = xx[10];
  xx[17] = xx[10];
  xx[18] = xx[3];
  xx[1] = xx[12 + (partialType)];
  xx[3] = xx[11] * xx[5];
  xx[5] = sqrt(xx[9] * xx[9] + xx[3] * xx[3]);
  xx[7] = xx[5] == 0.0 ? 0.0 : xx[9] / xx[5];
  xx[12] = xx[10];
  xx[13] = xx[10];
  xx[14] = xx[10];
  xx[15] = xx[7];
  xx[16] = xx[10];
  xx[17] = xx[10];
  xx[7] = xx[11 + (partialType)];
  xx[12] = xx[10];
  xx[13] = xx[10];
  xx[14] = xx[10];
  xx[15] = xx[10];
  xx[16] = xx[10];
  xx[17] = xx[10];
  xx[8] = xx[11 + (partialType)];
  xx[9] = xx[5] == 0.0 ? 0.0 : xx[3] / xx[5];
  xx[12] = xx[4];
  xx[13] = xx[4];
  xx[14] = xx[10];
  xx[15] = xx[9];
  xx[16] = xx[10];
  xx[17] = xx[10];
  xx[0] = xx[11 + (partialType)];
  matrix[0] = xx[6];
  matrix[1] = xx[2];
  matrix[2] = xx[1];
  matrix[3] = xx[7];
  matrix[4] = xx[8];
  matrix[5] = xx[0];
  matrix[6] = xx[8];
  matrix[7] = xx[8];
  matrix[8] = xx[8];
}

void dempsystest_bjorn_59d2bdba_5_computePosDofBlendMatrix(const void *mech,
  size_t stageIdx, size_t primIdx, const double *state, int partialType, double *
  matrix)
{
  (void) mech;
  (void) stageIdx;
  (void) primIdx;
  (void) state;
  (void) partialType;
  (void) matrix;
  switch ((stageIdx * 6 + primIdx))
  {
   case 3:
    computePosDofBlendMatrix_0_3(state, partialType, matrix);
    break;

   case 6:
    computePosDofBlendMatrix_1_0(state, partialType, matrix);
    break;

   case 12:
    computePosDofBlendMatrix_2_0(state, partialType, matrix);
    break;

   case 18:
    computePosDofBlendMatrix_3_0(state, partialType, matrix);
    break;

   case 36:
    computePosDofBlendMatrix_6_0(state, partialType, matrix);
    break;

   case 42:
    computePosDofBlendMatrix_7_0(state, partialType, matrix);
    break;

   case 48:
    computePosDofBlendMatrix_8_0(state, partialType, matrix);
    break;

   case 54:
    computePosDofBlendMatrix_9_0(state, partialType, matrix);
    break;

   case 60:
    computePosDofBlendMatrix_10_0(state, partialType, matrix);
    break;

   case 66:
    computePosDofBlendMatrix_11_0(state, partialType, matrix);
    break;

   case 72:
    computePosDofBlendMatrix_12_0(state, partialType, matrix);
    break;

   case 78:
    computePosDofBlendMatrix_13_0(state, partialType, matrix);
    break;

   case 96:
    computePosDofBlendMatrix_16_0(state, partialType, matrix);
    break;

   case 102:
    computePosDofBlendMatrix_17_0(state, partialType, matrix);
    break;

   case 108:
    computePosDofBlendMatrix_18_0(state, partialType, matrix);
    break;

   case 114:
    computePosDofBlendMatrix_19_0(state, partialType, matrix);
    break;

   case 120:
    computePosDofBlendMatrix_20_0(state, partialType, matrix);
    break;
  }
}

static void computeVelDofBlendMatrix_0_3(const double *state, int partialType,
  double *matrix)
{
  double xx[15];
  (void) state;
  xx[0] = 0.0;
  xx[1] = 0.0;
  xx[2] = 1.0;
  xx[4] = xx[1];
  xx[5] = xx[2];
  xx[6] = xx[1];
  xx[7] = xx[2];
  xx[8] = xx[1];
  xx[9] = xx[2];
  xx[10] = xx[3 + (partialType)];
  xx[4] = xx[2];
  xx[5] = xx[1];
  xx[6] = xx[1];
  xx[7] = xx[1];
  xx[8] = xx[2];
  xx[9] = xx[1];
  xx[11] = xx[3 + (partialType)];
  xx[4] = xx[1];
  xx[5] = xx[1];
  xx[6] = xx[2];
  xx[7] = xx[1];
  xx[8] = xx[1];
  xx[9] = xx[1];
  xx[12] = xx[3 + (partialType)];
  xx[4] = xx[1];
  xx[5] = xx[1];
  xx[6] = xx[1];
  xx[7] = xx[1];
  xx[8] = xx[1];
  xx[9] = xx[1];
  xx[13] = xx[3 + (partialType)];
  xx[4] = xx[1];
  xx[5] = xx[1];
  xx[6] = xx[1];
  xx[7] = xx[2];
  xx[8] = xx[1];
  xx[9] = xx[1];
  xx[14] = xx[3 + (partialType)];
  xx[4] = xx[2];
  xx[5] = xx[2];
  xx[6] = xx[1];
  xx[7] = xx[1];
  xx[8] = xx[1];
  xx[9] = xx[1];
  xx[0] = xx[3 + (partialType)];
  matrix[0] = xx[10];
  matrix[1] = xx[11];
  matrix[2] = xx[12];
  matrix[3] = xx[13];
  matrix[4] = xx[14];
  matrix[5] = xx[0];
  matrix[6] = xx[13];
  matrix[7] = xx[13];
  matrix[8] = xx[13];
}

static void computeVelDofBlendMatrix_1_0(const double *state, int partialType,
  double *matrix)
{
  double xx[15];
  (void) state;
  xx[0] = 0.0;
  xx[1] = 0.0;
  xx[2] = 1.0;
  xx[4] = xx[1];
  xx[5] = xx[2];
  xx[6] = xx[1];
  xx[7] = xx[2];
  xx[8] = xx[1];
  xx[9] = xx[2];
  xx[10] = xx[3 + (partialType)];
  xx[4] = xx[2];
  xx[5] = xx[1];
  xx[6] = xx[1];
  xx[7] = xx[1];
  xx[8] = xx[2];
  xx[9] = xx[1];
  xx[11] = xx[3 + (partialType)];
  xx[4] = xx[1];
  xx[5] = xx[1];
  xx[6] = xx[2];
  xx[7] = xx[1];
  xx[8] = xx[1];
  xx[9] = xx[1];
  xx[12] = xx[3 + (partialType)];
  xx[4] = xx[1];
  xx[5] = xx[1];
  xx[6] = xx[1];
  xx[7] = xx[1];
  xx[8] = xx[1];
  xx[9] = xx[1];
  xx[13] = xx[3 + (partialType)];
  xx[4] = xx[1];
  xx[5] = xx[1];
  xx[6] = xx[1];
  xx[7] = xx[2];
  xx[8] = xx[1];
  xx[9] = xx[1];
  xx[14] = xx[3 + (partialType)];
  xx[4] = xx[2];
  xx[5] = xx[2];
  xx[6] = xx[1];
  xx[7] = xx[1];
  xx[8] = xx[1];
  xx[9] = xx[1];
  xx[0] = xx[3 + (partialType)];
  matrix[0] = xx[10];
  matrix[1] = xx[11];
  matrix[2] = xx[12];
  matrix[3] = xx[13];
  matrix[4] = xx[14];
  matrix[5] = xx[0];
  matrix[6] = xx[13];
  matrix[7] = xx[13];
  matrix[8] = xx[13];
}

static void computeVelDofBlendMatrix_2_0(const double *state, int partialType,
  double *matrix)
{
  double xx[15];
  (void) state;
  xx[0] = 0.0;
  xx[1] = 0.0;
  xx[2] = 1.0;
  xx[4] = xx[1];
  xx[5] = xx[2];
  xx[6] = xx[1];
  xx[7] = xx[2];
  xx[8] = xx[1];
  xx[9] = xx[2];
  xx[10] = xx[3 + (partialType)];
  xx[4] = xx[2];
  xx[5] = xx[1];
  xx[6] = xx[1];
  xx[7] = xx[1];
  xx[8] = xx[2];
  xx[9] = xx[1];
  xx[11] = xx[3 + (partialType)];
  xx[4] = xx[1];
  xx[5] = xx[1];
  xx[6] = xx[2];
  xx[7] = xx[1];
  xx[8] = xx[1];
  xx[9] = xx[1];
  xx[12] = xx[3 + (partialType)];
  xx[4] = xx[1];
  xx[5] = xx[1];
  xx[6] = xx[1];
  xx[7] = xx[1];
  xx[8] = xx[1];
  xx[9] = xx[1];
  xx[13] = xx[3 + (partialType)];
  xx[4] = xx[1];
  xx[5] = xx[1];
  xx[6] = xx[1];
  xx[7] = xx[2];
  xx[8] = xx[1];
  xx[9] = xx[1];
  xx[14] = xx[3 + (partialType)];
  xx[4] = xx[2];
  xx[5] = xx[2];
  xx[6] = xx[1];
  xx[7] = xx[1];
  xx[8] = xx[1];
  xx[9] = xx[1];
  xx[0] = xx[3 + (partialType)];
  matrix[0] = xx[10];
  matrix[1] = xx[11];
  matrix[2] = xx[12];
  matrix[3] = xx[13];
  matrix[4] = xx[14];
  matrix[5] = xx[0];
  matrix[6] = xx[13];
  matrix[7] = xx[13];
  matrix[8] = xx[13];
}

static void computeVelDofBlendMatrix_3_0(const double *state, int partialType,
  double *matrix)
{
  double xx[15];
  (void) state;
  xx[0] = 0.0;
  xx[1] = 0.0;
  xx[2] = 1.0;
  xx[4] = xx[1];
  xx[5] = xx[2];
  xx[6] = xx[1];
  xx[7] = xx[2];
  xx[8] = xx[1];
  xx[9] = xx[2];
  xx[10] = xx[3 + (partialType)];
  xx[4] = xx[2];
  xx[5] = xx[1];
  xx[6] = xx[1];
  xx[7] = xx[1];
  xx[8] = xx[2];
  xx[9] = xx[1];
  xx[11] = xx[3 + (partialType)];
  xx[4] = xx[1];
  xx[5] = xx[1];
  xx[6] = xx[2];
  xx[7] = xx[1];
  xx[8] = xx[1];
  xx[9] = xx[1];
  xx[12] = xx[3 + (partialType)];
  xx[4] = xx[1];
  xx[5] = xx[1];
  xx[6] = xx[1];
  xx[7] = xx[1];
  xx[8] = xx[1];
  xx[9] = xx[1];
  xx[13] = xx[3 + (partialType)];
  xx[4] = xx[1];
  xx[5] = xx[1];
  xx[6] = xx[1];
  xx[7] = xx[2];
  xx[8] = xx[1];
  xx[9] = xx[1];
  xx[14] = xx[3 + (partialType)];
  xx[4] = xx[2];
  xx[5] = xx[2];
  xx[6] = xx[1];
  xx[7] = xx[1];
  xx[8] = xx[1];
  xx[9] = xx[1];
  xx[0] = xx[3 + (partialType)];
  matrix[0] = xx[10];
  matrix[1] = xx[11];
  matrix[2] = xx[12];
  matrix[3] = xx[13];
  matrix[4] = xx[14];
  matrix[5] = xx[0];
  matrix[6] = xx[13];
  matrix[7] = xx[13];
  matrix[8] = xx[13];
}

static void computeVelDofBlendMatrix_6_0(const double *state, int partialType,
  double *matrix)
{
  double xx[15];
  (void) state;
  xx[0] = 0.0;
  xx[1] = 0.0;
  xx[2] = 1.0;
  xx[4] = xx[1];
  xx[5] = xx[2];
  xx[6] = xx[1];
  xx[7] = xx[2];
  xx[8] = xx[1];
  xx[9] = xx[2];
  xx[10] = xx[3 + (partialType)];
  xx[4] = xx[2];
  xx[5] = xx[1];
  xx[6] = xx[1];
  xx[7] = xx[1];
  xx[8] = xx[2];
  xx[9] = xx[1];
  xx[11] = xx[3 + (partialType)];
  xx[4] = xx[1];
  xx[5] = xx[1];
  xx[6] = xx[2];
  xx[7] = xx[1];
  xx[8] = xx[1];
  xx[9] = xx[1];
  xx[12] = xx[3 + (partialType)];
  xx[4] = xx[1];
  xx[5] = xx[1];
  xx[6] = xx[1];
  xx[7] = xx[1];
  xx[8] = xx[1];
  xx[9] = xx[1];
  xx[13] = xx[3 + (partialType)];
  xx[4] = xx[1];
  xx[5] = xx[1];
  xx[6] = xx[1];
  xx[7] = xx[2];
  xx[8] = xx[1];
  xx[9] = xx[1];
  xx[14] = xx[3 + (partialType)];
  xx[4] = xx[2];
  xx[5] = xx[2];
  xx[6] = xx[1];
  xx[7] = xx[1];
  xx[8] = xx[1];
  xx[9] = xx[1];
  xx[0] = xx[3 + (partialType)];
  matrix[0] = xx[10];
  matrix[1] = xx[11];
  matrix[2] = xx[12];
  matrix[3] = xx[13];
  matrix[4] = xx[14];
  matrix[5] = xx[0];
  matrix[6] = xx[13];
  matrix[7] = xx[13];
  matrix[8] = xx[13];
}

static void computeVelDofBlendMatrix_7_0(const double *state, int partialType,
  double *matrix)
{
  double xx[15];
  (void) state;
  xx[0] = 0.0;
  xx[1] = 0.0;
  xx[2] = 1.0;
  xx[4] = xx[1];
  xx[5] = xx[2];
  xx[6] = xx[1];
  xx[7] = xx[2];
  xx[8] = xx[1];
  xx[9] = xx[2];
  xx[10] = xx[3 + (partialType)];
  xx[4] = xx[2];
  xx[5] = xx[1];
  xx[6] = xx[1];
  xx[7] = xx[1];
  xx[8] = xx[2];
  xx[9] = xx[1];
  xx[11] = xx[3 + (partialType)];
  xx[4] = xx[1];
  xx[5] = xx[1];
  xx[6] = xx[2];
  xx[7] = xx[1];
  xx[8] = xx[1];
  xx[9] = xx[1];
  xx[12] = xx[3 + (partialType)];
  xx[4] = xx[1];
  xx[5] = xx[1];
  xx[6] = xx[1];
  xx[7] = xx[1];
  xx[8] = xx[1];
  xx[9] = xx[1];
  xx[13] = xx[3 + (partialType)];
  xx[4] = xx[1];
  xx[5] = xx[1];
  xx[6] = xx[1];
  xx[7] = xx[2];
  xx[8] = xx[1];
  xx[9] = xx[1];
  xx[14] = xx[3 + (partialType)];
  xx[4] = xx[2];
  xx[5] = xx[2];
  xx[6] = xx[1];
  xx[7] = xx[1];
  xx[8] = xx[1];
  xx[9] = xx[1];
  xx[0] = xx[3 + (partialType)];
  matrix[0] = xx[10];
  matrix[1] = xx[11];
  matrix[2] = xx[12];
  matrix[3] = xx[13];
  matrix[4] = xx[14];
  matrix[5] = xx[0];
  matrix[6] = xx[13];
  matrix[7] = xx[13];
  matrix[8] = xx[13];
}

static void computeVelDofBlendMatrix_8_0(const double *state, int partialType,
  double *matrix)
{
  double xx[15];
  (void) state;
  xx[0] = 0.0;
  xx[1] = 0.0;
  xx[2] = 1.0;
  xx[4] = xx[1];
  xx[5] = xx[2];
  xx[6] = xx[1];
  xx[7] = xx[2];
  xx[8] = xx[1];
  xx[9] = xx[2];
  xx[10] = xx[3 + (partialType)];
  xx[4] = xx[2];
  xx[5] = xx[1];
  xx[6] = xx[1];
  xx[7] = xx[1];
  xx[8] = xx[2];
  xx[9] = xx[1];
  xx[11] = xx[3 + (partialType)];
  xx[4] = xx[1];
  xx[5] = xx[1];
  xx[6] = xx[2];
  xx[7] = xx[1];
  xx[8] = xx[1];
  xx[9] = xx[1];
  xx[12] = xx[3 + (partialType)];
  xx[4] = xx[1];
  xx[5] = xx[1];
  xx[6] = xx[1];
  xx[7] = xx[1];
  xx[8] = xx[1];
  xx[9] = xx[1];
  xx[13] = xx[3 + (partialType)];
  xx[4] = xx[1];
  xx[5] = xx[1];
  xx[6] = xx[1];
  xx[7] = xx[2];
  xx[8] = xx[1];
  xx[9] = xx[1];
  xx[14] = xx[3 + (partialType)];
  xx[4] = xx[2];
  xx[5] = xx[2];
  xx[6] = xx[1];
  xx[7] = xx[1];
  xx[8] = xx[1];
  xx[9] = xx[1];
  xx[0] = xx[3 + (partialType)];
  matrix[0] = xx[10];
  matrix[1] = xx[11];
  matrix[2] = xx[12];
  matrix[3] = xx[13];
  matrix[4] = xx[14];
  matrix[5] = xx[0];
  matrix[6] = xx[13];
  matrix[7] = xx[13];
  matrix[8] = xx[13];
}

static void computeVelDofBlendMatrix_9_0(const double *state, int partialType,
  double *matrix)
{
  double xx[15];
  (void) state;
  xx[0] = 0.0;
  xx[1] = 0.0;
  xx[2] = 1.0;
  xx[4] = xx[1];
  xx[5] = xx[2];
  xx[6] = xx[1];
  xx[7] = xx[2];
  xx[8] = xx[1];
  xx[9] = xx[2];
  xx[10] = xx[3 + (partialType)];
  xx[4] = xx[2];
  xx[5] = xx[1];
  xx[6] = xx[1];
  xx[7] = xx[1];
  xx[8] = xx[2];
  xx[9] = xx[1];
  xx[11] = xx[3 + (partialType)];
  xx[4] = xx[1];
  xx[5] = xx[1];
  xx[6] = xx[2];
  xx[7] = xx[1];
  xx[8] = xx[1];
  xx[9] = xx[1];
  xx[12] = xx[3 + (partialType)];
  xx[4] = xx[1];
  xx[5] = xx[1];
  xx[6] = xx[1];
  xx[7] = xx[1];
  xx[8] = xx[1];
  xx[9] = xx[1];
  xx[13] = xx[3 + (partialType)];
  xx[4] = xx[1];
  xx[5] = xx[1];
  xx[6] = xx[1];
  xx[7] = xx[2];
  xx[8] = xx[1];
  xx[9] = xx[1];
  xx[14] = xx[3 + (partialType)];
  xx[4] = xx[2];
  xx[5] = xx[2];
  xx[6] = xx[1];
  xx[7] = xx[1];
  xx[8] = xx[1];
  xx[9] = xx[1];
  xx[0] = xx[3 + (partialType)];
  matrix[0] = xx[10];
  matrix[1] = xx[11];
  matrix[2] = xx[12];
  matrix[3] = xx[13];
  matrix[4] = xx[14];
  matrix[5] = xx[0];
  matrix[6] = xx[13];
  matrix[7] = xx[13];
  matrix[8] = xx[13];
}

static void computeVelDofBlendMatrix_10_0(const double *state, int partialType,
  double *matrix)
{
  double xx[15];
  (void) state;
  xx[0] = 0.0;
  xx[1] = 0.0;
  xx[2] = 1.0;
  xx[4] = xx[1];
  xx[5] = xx[2];
  xx[6] = xx[1];
  xx[7] = xx[2];
  xx[8] = xx[1];
  xx[9] = xx[2];
  xx[10] = xx[3 + (partialType)];
  xx[4] = xx[2];
  xx[5] = xx[1];
  xx[6] = xx[1];
  xx[7] = xx[1];
  xx[8] = xx[2];
  xx[9] = xx[1];
  xx[11] = xx[3 + (partialType)];
  xx[4] = xx[1];
  xx[5] = xx[1];
  xx[6] = xx[2];
  xx[7] = xx[1];
  xx[8] = xx[1];
  xx[9] = xx[1];
  xx[12] = xx[3 + (partialType)];
  xx[4] = xx[1];
  xx[5] = xx[1];
  xx[6] = xx[1];
  xx[7] = xx[1];
  xx[8] = xx[1];
  xx[9] = xx[1];
  xx[13] = xx[3 + (partialType)];
  xx[4] = xx[1];
  xx[5] = xx[1];
  xx[6] = xx[1];
  xx[7] = xx[2];
  xx[8] = xx[1];
  xx[9] = xx[1];
  xx[14] = xx[3 + (partialType)];
  xx[4] = xx[2];
  xx[5] = xx[2];
  xx[6] = xx[1];
  xx[7] = xx[1];
  xx[8] = xx[1];
  xx[9] = xx[1];
  xx[0] = xx[3 + (partialType)];
  matrix[0] = xx[10];
  matrix[1] = xx[11];
  matrix[2] = xx[12];
  matrix[3] = xx[13];
  matrix[4] = xx[14];
  matrix[5] = xx[0];
  matrix[6] = xx[13];
  matrix[7] = xx[13];
  matrix[8] = xx[13];
}

static void computeVelDofBlendMatrix_11_0(const double *state, int partialType,
  double *matrix)
{
  double xx[15];
  (void) state;
  xx[0] = 0.0;
  xx[1] = 0.0;
  xx[2] = 1.0;
  xx[4] = xx[1];
  xx[5] = xx[2];
  xx[6] = xx[1];
  xx[7] = xx[2];
  xx[8] = xx[1];
  xx[9] = xx[2];
  xx[10] = xx[3 + (partialType)];
  xx[4] = xx[2];
  xx[5] = xx[1];
  xx[6] = xx[1];
  xx[7] = xx[1];
  xx[8] = xx[2];
  xx[9] = xx[1];
  xx[11] = xx[3 + (partialType)];
  xx[4] = xx[1];
  xx[5] = xx[1];
  xx[6] = xx[2];
  xx[7] = xx[1];
  xx[8] = xx[1];
  xx[9] = xx[1];
  xx[12] = xx[3 + (partialType)];
  xx[4] = xx[1];
  xx[5] = xx[1];
  xx[6] = xx[1];
  xx[7] = xx[1];
  xx[8] = xx[1];
  xx[9] = xx[1];
  xx[13] = xx[3 + (partialType)];
  xx[4] = xx[1];
  xx[5] = xx[1];
  xx[6] = xx[1];
  xx[7] = xx[2];
  xx[8] = xx[1];
  xx[9] = xx[1];
  xx[14] = xx[3 + (partialType)];
  xx[4] = xx[2];
  xx[5] = xx[2];
  xx[6] = xx[1];
  xx[7] = xx[1];
  xx[8] = xx[1];
  xx[9] = xx[1];
  xx[0] = xx[3 + (partialType)];
  matrix[0] = xx[10];
  matrix[1] = xx[11];
  matrix[2] = xx[12];
  matrix[3] = xx[13];
  matrix[4] = xx[14];
  matrix[5] = xx[0];
  matrix[6] = xx[13];
  matrix[7] = xx[13];
  matrix[8] = xx[13];
}

static void computeVelDofBlendMatrix_12_0(const double *state, int partialType,
  double *matrix)
{
  double xx[15];
  (void) state;
  xx[0] = 0.0;
  xx[1] = 0.0;
  xx[2] = 1.0;
  xx[4] = xx[1];
  xx[5] = xx[2];
  xx[6] = xx[1];
  xx[7] = xx[2];
  xx[8] = xx[1];
  xx[9] = xx[2];
  xx[10] = xx[3 + (partialType)];
  xx[4] = xx[2];
  xx[5] = xx[1];
  xx[6] = xx[1];
  xx[7] = xx[1];
  xx[8] = xx[2];
  xx[9] = xx[1];
  xx[11] = xx[3 + (partialType)];
  xx[4] = xx[1];
  xx[5] = xx[1];
  xx[6] = xx[2];
  xx[7] = xx[1];
  xx[8] = xx[1];
  xx[9] = xx[1];
  xx[12] = xx[3 + (partialType)];
  xx[4] = xx[1];
  xx[5] = xx[1];
  xx[6] = xx[1];
  xx[7] = xx[1];
  xx[8] = xx[1];
  xx[9] = xx[1];
  xx[13] = xx[3 + (partialType)];
  xx[4] = xx[1];
  xx[5] = xx[1];
  xx[6] = xx[1];
  xx[7] = xx[2];
  xx[8] = xx[1];
  xx[9] = xx[1];
  xx[14] = xx[3 + (partialType)];
  xx[4] = xx[2];
  xx[5] = xx[2];
  xx[6] = xx[1];
  xx[7] = xx[1];
  xx[8] = xx[1];
  xx[9] = xx[1];
  xx[0] = xx[3 + (partialType)];
  matrix[0] = xx[10];
  matrix[1] = xx[11];
  matrix[2] = xx[12];
  matrix[3] = xx[13];
  matrix[4] = xx[14];
  matrix[5] = xx[0];
  matrix[6] = xx[13];
  matrix[7] = xx[13];
  matrix[8] = xx[13];
}

static void computeVelDofBlendMatrix_13_0(const double *state, int partialType,
  double *matrix)
{
  double xx[15];
  (void) state;
  xx[0] = 0.0;
  xx[1] = 0.0;
  xx[2] = 1.0;
  xx[4] = xx[1];
  xx[5] = xx[2];
  xx[6] = xx[1];
  xx[7] = xx[2];
  xx[8] = xx[1];
  xx[9] = xx[2];
  xx[10] = xx[3 + (partialType)];
  xx[4] = xx[2];
  xx[5] = xx[1];
  xx[6] = xx[1];
  xx[7] = xx[1];
  xx[8] = xx[2];
  xx[9] = xx[1];
  xx[11] = xx[3 + (partialType)];
  xx[4] = xx[1];
  xx[5] = xx[1];
  xx[6] = xx[2];
  xx[7] = xx[1];
  xx[8] = xx[1];
  xx[9] = xx[1];
  xx[12] = xx[3 + (partialType)];
  xx[4] = xx[1];
  xx[5] = xx[1];
  xx[6] = xx[1];
  xx[7] = xx[1];
  xx[8] = xx[1];
  xx[9] = xx[1];
  xx[13] = xx[3 + (partialType)];
  xx[4] = xx[1];
  xx[5] = xx[1];
  xx[6] = xx[1];
  xx[7] = xx[2];
  xx[8] = xx[1];
  xx[9] = xx[1];
  xx[14] = xx[3 + (partialType)];
  xx[4] = xx[2];
  xx[5] = xx[2];
  xx[6] = xx[1];
  xx[7] = xx[1];
  xx[8] = xx[1];
  xx[9] = xx[1];
  xx[0] = xx[3 + (partialType)];
  matrix[0] = xx[10];
  matrix[1] = xx[11];
  matrix[2] = xx[12];
  matrix[3] = xx[13];
  matrix[4] = xx[14];
  matrix[5] = xx[0];
  matrix[6] = xx[13];
  matrix[7] = xx[13];
  matrix[8] = xx[13];
}

static void computeVelDofBlendMatrix_16_0(const double *state, int partialType,
  double *matrix)
{
  double xx[15];
  (void) state;
  xx[0] = 0.0;
  xx[1] = 0.0;
  xx[2] = 1.0;
  xx[4] = xx[1];
  xx[5] = xx[2];
  xx[6] = xx[1];
  xx[7] = xx[2];
  xx[8] = xx[1];
  xx[9] = xx[2];
  xx[10] = xx[3 + (partialType)];
  xx[4] = xx[2];
  xx[5] = xx[1];
  xx[6] = xx[1];
  xx[7] = xx[1];
  xx[8] = xx[2];
  xx[9] = xx[1];
  xx[11] = xx[3 + (partialType)];
  xx[4] = xx[1];
  xx[5] = xx[1];
  xx[6] = xx[2];
  xx[7] = xx[1];
  xx[8] = xx[1];
  xx[9] = xx[1];
  xx[12] = xx[3 + (partialType)];
  xx[4] = xx[1];
  xx[5] = xx[1];
  xx[6] = xx[1];
  xx[7] = xx[1];
  xx[8] = xx[1];
  xx[9] = xx[1];
  xx[13] = xx[3 + (partialType)];
  xx[4] = xx[1];
  xx[5] = xx[1];
  xx[6] = xx[1];
  xx[7] = xx[2];
  xx[8] = xx[1];
  xx[9] = xx[1];
  xx[14] = xx[3 + (partialType)];
  xx[4] = xx[2];
  xx[5] = xx[2];
  xx[6] = xx[1];
  xx[7] = xx[1];
  xx[8] = xx[1];
  xx[9] = xx[1];
  xx[0] = xx[3 + (partialType)];
  matrix[0] = xx[10];
  matrix[1] = xx[11];
  matrix[2] = xx[12];
  matrix[3] = xx[13];
  matrix[4] = xx[14];
  matrix[5] = xx[0];
  matrix[6] = xx[13];
  matrix[7] = xx[13];
  matrix[8] = xx[13];
}

static void computeVelDofBlendMatrix_17_0(const double *state, int partialType,
  double *matrix)
{
  double xx[15];
  (void) state;
  xx[0] = 0.0;
  xx[1] = 0.0;
  xx[2] = 1.0;
  xx[4] = xx[1];
  xx[5] = xx[2];
  xx[6] = xx[1];
  xx[7] = xx[2];
  xx[8] = xx[1];
  xx[9] = xx[2];
  xx[10] = xx[3 + (partialType)];
  xx[4] = xx[2];
  xx[5] = xx[1];
  xx[6] = xx[1];
  xx[7] = xx[1];
  xx[8] = xx[2];
  xx[9] = xx[1];
  xx[11] = xx[3 + (partialType)];
  xx[4] = xx[1];
  xx[5] = xx[1];
  xx[6] = xx[2];
  xx[7] = xx[1];
  xx[8] = xx[1];
  xx[9] = xx[1];
  xx[12] = xx[3 + (partialType)];
  xx[4] = xx[1];
  xx[5] = xx[1];
  xx[6] = xx[1];
  xx[7] = xx[1];
  xx[8] = xx[1];
  xx[9] = xx[1];
  xx[13] = xx[3 + (partialType)];
  xx[4] = xx[1];
  xx[5] = xx[1];
  xx[6] = xx[1];
  xx[7] = xx[2];
  xx[8] = xx[1];
  xx[9] = xx[1];
  xx[14] = xx[3 + (partialType)];
  xx[4] = xx[2];
  xx[5] = xx[2];
  xx[6] = xx[1];
  xx[7] = xx[1];
  xx[8] = xx[1];
  xx[9] = xx[1];
  xx[0] = xx[3 + (partialType)];
  matrix[0] = xx[10];
  matrix[1] = xx[11];
  matrix[2] = xx[12];
  matrix[3] = xx[13];
  matrix[4] = xx[14];
  matrix[5] = xx[0];
  matrix[6] = xx[13];
  matrix[7] = xx[13];
  matrix[8] = xx[13];
}

static void computeVelDofBlendMatrix_18_0(const double *state, int partialType,
  double *matrix)
{
  double xx[15];
  (void) state;
  xx[0] = 0.0;
  xx[1] = 0.0;
  xx[2] = 1.0;
  xx[4] = xx[1];
  xx[5] = xx[2];
  xx[6] = xx[1];
  xx[7] = xx[2];
  xx[8] = xx[1];
  xx[9] = xx[2];
  xx[10] = xx[3 + (partialType)];
  xx[4] = xx[2];
  xx[5] = xx[1];
  xx[6] = xx[1];
  xx[7] = xx[1];
  xx[8] = xx[2];
  xx[9] = xx[1];
  xx[11] = xx[3 + (partialType)];
  xx[4] = xx[1];
  xx[5] = xx[1];
  xx[6] = xx[2];
  xx[7] = xx[1];
  xx[8] = xx[1];
  xx[9] = xx[1];
  xx[12] = xx[3 + (partialType)];
  xx[4] = xx[1];
  xx[5] = xx[1];
  xx[6] = xx[1];
  xx[7] = xx[1];
  xx[8] = xx[1];
  xx[9] = xx[1];
  xx[13] = xx[3 + (partialType)];
  xx[4] = xx[1];
  xx[5] = xx[1];
  xx[6] = xx[1];
  xx[7] = xx[2];
  xx[8] = xx[1];
  xx[9] = xx[1];
  xx[14] = xx[3 + (partialType)];
  xx[4] = xx[2];
  xx[5] = xx[2];
  xx[6] = xx[1];
  xx[7] = xx[1];
  xx[8] = xx[1];
  xx[9] = xx[1];
  xx[0] = xx[3 + (partialType)];
  matrix[0] = xx[10];
  matrix[1] = xx[11];
  matrix[2] = xx[12];
  matrix[3] = xx[13];
  matrix[4] = xx[14];
  matrix[5] = xx[0];
  matrix[6] = xx[13];
  matrix[7] = xx[13];
  matrix[8] = xx[13];
}

static void computeVelDofBlendMatrix_19_0(const double *state, int partialType,
  double *matrix)
{
  double xx[15];
  (void) state;
  xx[0] = 0.0;
  xx[1] = 0.0;
  xx[2] = 1.0;
  xx[4] = xx[1];
  xx[5] = xx[2];
  xx[6] = xx[1];
  xx[7] = xx[2];
  xx[8] = xx[1];
  xx[9] = xx[2];
  xx[10] = xx[3 + (partialType)];
  xx[4] = xx[2];
  xx[5] = xx[1];
  xx[6] = xx[1];
  xx[7] = xx[1];
  xx[8] = xx[2];
  xx[9] = xx[1];
  xx[11] = xx[3 + (partialType)];
  xx[4] = xx[1];
  xx[5] = xx[1];
  xx[6] = xx[2];
  xx[7] = xx[1];
  xx[8] = xx[1];
  xx[9] = xx[1];
  xx[12] = xx[3 + (partialType)];
  xx[4] = xx[1];
  xx[5] = xx[1];
  xx[6] = xx[1];
  xx[7] = xx[1];
  xx[8] = xx[1];
  xx[9] = xx[1];
  xx[13] = xx[3 + (partialType)];
  xx[4] = xx[1];
  xx[5] = xx[1];
  xx[6] = xx[1];
  xx[7] = xx[2];
  xx[8] = xx[1];
  xx[9] = xx[1];
  xx[14] = xx[3 + (partialType)];
  xx[4] = xx[2];
  xx[5] = xx[2];
  xx[6] = xx[1];
  xx[7] = xx[1];
  xx[8] = xx[1];
  xx[9] = xx[1];
  xx[0] = xx[3 + (partialType)];
  matrix[0] = xx[10];
  matrix[1] = xx[11];
  matrix[2] = xx[12];
  matrix[3] = xx[13];
  matrix[4] = xx[14];
  matrix[5] = xx[0];
  matrix[6] = xx[13];
  matrix[7] = xx[13];
  matrix[8] = xx[13];
}

static void computeVelDofBlendMatrix_20_0(const double *state, int partialType,
  double *matrix)
{
  double xx[15];
  (void) state;
  xx[0] = 0.0;
  xx[1] = 0.0;
  xx[2] = 1.0;
  xx[4] = xx[1];
  xx[5] = xx[2];
  xx[6] = xx[1];
  xx[7] = xx[2];
  xx[8] = xx[1];
  xx[9] = xx[2];
  xx[10] = xx[3 + (partialType)];
  xx[4] = xx[2];
  xx[5] = xx[1];
  xx[6] = xx[1];
  xx[7] = xx[1];
  xx[8] = xx[2];
  xx[9] = xx[1];
  xx[11] = xx[3 + (partialType)];
  xx[4] = xx[1];
  xx[5] = xx[1];
  xx[6] = xx[2];
  xx[7] = xx[1];
  xx[8] = xx[1];
  xx[9] = xx[1];
  xx[12] = xx[3 + (partialType)];
  xx[4] = xx[1];
  xx[5] = xx[1];
  xx[6] = xx[1];
  xx[7] = xx[1];
  xx[8] = xx[1];
  xx[9] = xx[1];
  xx[13] = xx[3 + (partialType)];
  xx[4] = xx[1];
  xx[5] = xx[1];
  xx[6] = xx[1];
  xx[7] = xx[2];
  xx[8] = xx[1];
  xx[9] = xx[1];
  xx[14] = xx[3 + (partialType)];
  xx[4] = xx[2];
  xx[5] = xx[2];
  xx[6] = xx[1];
  xx[7] = xx[1];
  xx[8] = xx[1];
  xx[9] = xx[1];
  xx[0] = xx[3 + (partialType)];
  matrix[0] = xx[10];
  matrix[1] = xx[11];
  matrix[2] = xx[12];
  matrix[3] = xx[13];
  matrix[4] = xx[14];
  matrix[5] = xx[0];
  matrix[6] = xx[13];
  matrix[7] = xx[13];
  matrix[8] = xx[13];
}

void dempsystest_bjorn_59d2bdba_5_computeVelDofBlendMatrix(const void *mech,
  size_t stageIdx, size_t primIdx, const double *state, int partialType, double *
  matrix)
{
  (void) mech;
  (void) stageIdx;
  (void) primIdx;
  (void) state;
  (void) partialType;
  (void) matrix;
  switch ((stageIdx * 6 + primIdx))
  {
   case 3:
    computeVelDofBlendMatrix_0_3(state, partialType, matrix);
    break;

   case 6:
    computeVelDofBlendMatrix_1_0(state, partialType, matrix);
    break;

   case 12:
    computeVelDofBlendMatrix_2_0(state, partialType, matrix);
    break;

   case 18:
    computeVelDofBlendMatrix_3_0(state, partialType, matrix);
    break;

   case 36:
    computeVelDofBlendMatrix_6_0(state, partialType, matrix);
    break;

   case 42:
    computeVelDofBlendMatrix_7_0(state, partialType, matrix);
    break;

   case 48:
    computeVelDofBlendMatrix_8_0(state, partialType, matrix);
    break;

   case 54:
    computeVelDofBlendMatrix_9_0(state, partialType, matrix);
    break;

   case 60:
    computeVelDofBlendMatrix_10_0(state, partialType, matrix);
    break;

   case 66:
    computeVelDofBlendMatrix_11_0(state, partialType, matrix);
    break;

   case 72:
    computeVelDofBlendMatrix_12_0(state, partialType, matrix);
    break;

   case 78:
    computeVelDofBlendMatrix_13_0(state, partialType, matrix);
    break;

   case 96:
    computeVelDofBlendMatrix_16_0(state, partialType, matrix);
    break;

   case 102:
    computeVelDofBlendMatrix_17_0(state, partialType, matrix);
    break;

   case 108:
    computeVelDofBlendMatrix_18_0(state, partialType, matrix);
    break;

   case 114:
    computeVelDofBlendMatrix_19_0(state, partialType, matrix);
    break;

   case 120:
    computeVelDofBlendMatrix_20_0(state, partialType, matrix);
    break;
  }
}

static void projectPartiallyTargetedPos_0_3(const double *origState, int
  partialType, double *state)
{
  boolean_T bb[1];
  int ii[3];
  double xx[24];
  xx[0] = state[4] * state[6];
  xx[1] = state[3] * state[5];
  xx[2] = 2.0;
  xx[3] = (xx[0] + xx[1]) * xx[2];
  xx[4] = fabs(xx[3]) > 1.0 ? atan2(xx[3], 0.0) : asin(xx[3]);
  xx[5] = origState[4] * origState[6];
  xx[6] = origState[3] * origState[5];
  xx[7] = (xx[5] + xx[6]) * xx[2];
  xx[8] = fabs(xx[7]) > 1.0 ? atan2(xx[7], 0.0) : asin(xx[7]);
  xx[9] = xx[4];
  xx[10] = xx[4];
  xx[11] = xx[8];
  xx[12] = xx[8];
  xx[13] = xx[4];
  xx[14] = xx[4];
  xx[15] = xx[8];
  xx[16] = xx[9 + (partialType)];
  xx[9] = cos(xx[16]);
  xx[10] = 0.99999999999999;
  xx[11] = fabs(xx[3]) - xx[10];
  if (xx[11] < 0.0)
    ii[0] = -1;
  else if (xx[11] > 0.0)
    ii[0] = +1;
  else
    ii[0] = 0;
  ii[1] = ii[0];
  if (0 > ii[1])
    ii[1] = 0;
  bb[0] = !(ii[1] == 1);
  xx[3] = state[4] * state[5];
  xx[11] = state[3] * state[6];
  xx[12] = state[3] * state[3];
  xx[13] = 1.0;
  xx[14] = (xx[12] + state[4] * state[4]) * xx[2] - xx[13];
  xx[15] = - (xx[2] * (xx[3] - xx[11]));
  xx[14] = (xx[15] == 0.0 && xx[14] == 0.0) ? 0.0 : atan2(xx[15], xx[14]);
  if (xx[4] < 0.0)
    xx[15] = -1.0;
  else if (xx[4] > 0.0)
    xx[15] = +1.0;
  else
    xx[15] = 0.0;
  xx[4] = (xx[12] + state[6] * state[6]) * xx[2] - xx[13];
  xx[17] = - (xx[2] * (state[5] * state[6] - state[3] * state[4]));
  xx[4] = (xx[17] == 0.0 && xx[4] == 0.0) ? 0.0 : atan2(xx[17], xx[4]);
  xx[12] = 0.5;
  xx[17] = - (xx[2] * (xx[0] - xx[1]) * xx[15]);
  xx[18] = (xx[3] + xx[11]) * xx[2] * xx[15];
  xx[17] = (xx[18] == 0.0 && xx[17] == 0.0) ? 0.0 : atan2(xx[18], xx[17]);
  xx[0] = bb[0] ? xx[4] : xx[12] * xx[17];
  xx[1] = bb[0] ? xx[14] : xx[15] * xx[0];
  xx[3] = fabs(xx[7]) - xx[10];
  if (xx[3] < 0.0)
    ii[0] = -1;
  else if (xx[3] > 0.0)
    ii[0] = +1;
  else
    ii[0] = 0;
  ii[1] = ii[0];
  if (0 > ii[1])
    ii[1] = 0;
  bb[0] = !(ii[1] == 1);
  xx[3] = origState[4] * origState[5];
  xx[4] = origState[3] * origState[6];
  xx[7] = origState[3] * origState[3];
  xx[10] = (xx[7] + origState[4] * origState[4]) * xx[2] - xx[13];
  xx[11] = - (xx[2] * (xx[3] - xx[4]));
  xx[10] = (xx[11] == 0.0 && xx[10] == 0.0) ? 0.0 : atan2(xx[11], xx[10]);
  if (xx[8] < 0.0)
    xx[11] = -1.0;
  else if (xx[8] > 0.0)
    xx[11] = +1.0;
  else
    xx[11] = 0.0;
  xx[8] = (xx[7] + origState[6] * origState[6]) * xx[2] - xx[13];
  xx[14] = - (xx[2] * (origState[5] * origState[6] - origState[3] * origState[4]));
  xx[8] = (xx[14] == 0.0 && xx[8] == 0.0) ? 0.0 : atan2(xx[14], xx[8]);
  xx[7] = - (xx[2] * (xx[5] - xx[6]) * xx[11]);
  xx[13] = (xx[3] + xx[4]) * xx[2] * xx[11];
  xx[7] = (xx[13] == 0.0 && xx[7] == 0.0) ? 0.0 : atan2(xx[13], xx[7]);
  xx[2] = bb[0] ? xx[8] : xx[12] * xx[7];
  xx[3] = bb[0] ? xx[10] : xx[11] * xx[2];
  xx[17] = xx[1];
  xx[18] = xx[1];
  xx[19] = xx[1];
  xx[20] = xx[1];
  xx[21] = xx[3];
  xx[22] = xx[3];
  xx[23] = xx[3];
  xx[1] = xx[17 + (partialType)];
  xx[3] = cos(xx[1]);
  xx[4] = sin(xx[1]);
  xx[1] = sin(xx[16]);
  xx[10] = xx[0];
  xx[11] = xx[2];
  xx[12] = xx[0];
  xx[13] = xx[2];
  xx[14] = xx[0];
  xx[15] = xx[2];
  xx[16] = xx[0];
  xx[0] = xx[10 + (partialType)];
  xx[2] = cos(xx[0]);
  xx[5] = sin(xx[0]);
  xx[0] = xx[3] * xx[5];
  xx[6] = xx[3] * xx[2];
  xx[10] = xx[9] * xx[3];
  xx[11] = - (xx[9] * xx[4]);
  xx[12] = xx[1];
  xx[13] = xx[2] * xx[4] + xx[0] * xx[1];
  xx[14] = xx[6] - xx[1] * xx[5] * xx[4];
  xx[15] = - (xx[9] * xx[5]);
  xx[16] = xx[4] * xx[5] - xx[6] * xx[1];
  xx[17] = xx[0] + xx[2] * xx[1] * xx[4];
  xx[18] = xx[9] * xx[2];
  pm_math_Quaternion_Matrix3x3Ctor_ra(xx + 10, xx + 0);
  state[3] = xx[0];
  state[4] = xx[1];
  state[5] = xx[2];
  state[6] = xx[3];
}

static void projectPartiallyTargetedPos_1_0(const double *origState, int
  partialType, double *state)
{
  boolean_T bb[1];
  int ii[3];
  double xx[24];
  xx[0] = state[14] * state[16];
  xx[1] = state[13] * state[15];
  xx[2] = 2.0;
  xx[3] = (xx[0] + xx[1]) * xx[2];
  xx[4] = fabs(xx[3]) > 1.0 ? atan2(xx[3], 0.0) : asin(xx[3]);
  xx[5] = origState[14] * origState[16];
  xx[6] = origState[13] * origState[15];
  xx[7] = (xx[5] + xx[6]) * xx[2];
  xx[8] = fabs(xx[7]) > 1.0 ? atan2(xx[7], 0.0) : asin(xx[7]);
  xx[9] = xx[4];
  xx[10] = xx[4];
  xx[11] = xx[8];
  xx[12] = xx[8];
  xx[13] = xx[4];
  xx[14] = xx[4];
  xx[15] = xx[8];
  xx[16] = xx[9 + (partialType)];
  xx[9] = cos(xx[16]);
  xx[10] = 0.99999999999999;
  xx[11] = fabs(xx[3]) - xx[10];
  if (xx[11] < 0.0)
    ii[0] = -1;
  else if (xx[11] > 0.0)
    ii[0] = +1;
  else
    ii[0] = 0;
  ii[1] = ii[0];
  if (0 > ii[1])
    ii[1] = 0;
  bb[0] = !(ii[1] == 1);
  xx[3] = state[14] * state[15];
  xx[11] = state[13] * state[16];
  xx[12] = state[13] * state[13];
  xx[13] = 1.0;
  xx[14] = (xx[12] + state[14] * state[14]) * xx[2] - xx[13];
  xx[15] = - (xx[2] * (xx[3] - xx[11]));
  xx[14] = (xx[15] == 0.0 && xx[14] == 0.0) ? 0.0 : atan2(xx[15], xx[14]);
  if (xx[4] < 0.0)
    xx[15] = -1.0;
  else if (xx[4] > 0.0)
    xx[15] = +1.0;
  else
    xx[15] = 0.0;
  xx[4] = (xx[12] + state[16] * state[16]) * xx[2] - xx[13];
  xx[17] = - (xx[2] * (state[15] * state[16] - state[13] * state[14]));
  xx[4] = (xx[17] == 0.0 && xx[4] == 0.0) ? 0.0 : atan2(xx[17], xx[4]);
  xx[12] = 0.5;
  xx[17] = - (xx[2] * (xx[0] - xx[1]) * xx[15]);
  xx[18] = (xx[3] + xx[11]) * xx[2] * xx[15];
  xx[17] = (xx[18] == 0.0 && xx[17] == 0.0) ? 0.0 : atan2(xx[18], xx[17]);
  xx[0] = bb[0] ? xx[4] : xx[12] * xx[17];
  xx[1] = bb[0] ? xx[14] : xx[15] * xx[0];
  xx[3] = fabs(xx[7]) - xx[10];
  if (xx[3] < 0.0)
    ii[0] = -1;
  else if (xx[3] > 0.0)
    ii[0] = +1;
  else
    ii[0] = 0;
  ii[1] = ii[0];
  if (0 > ii[1])
    ii[1] = 0;
  bb[0] = !(ii[1] == 1);
  xx[3] = origState[14] * origState[15];
  xx[4] = origState[13] * origState[16];
  xx[7] = origState[13] * origState[13];
  xx[10] = (xx[7] + origState[14] * origState[14]) * xx[2] - xx[13];
  xx[11] = - (xx[2] * (xx[3] - xx[4]));
  xx[10] = (xx[11] == 0.0 && xx[10] == 0.0) ? 0.0 : atan2(xx[11], xx[10]);
  if (xx[8] < 0.0)
    xx[11] = -1.0;
  else if (xx[8] > 0.0)
    xx[11] = +1.0;
  else
    xx[11] = 0.0;
  xx[8] = (xx[7] + origState[16] * origState[16]) * xx[2] - xx[13];
  xx[14] = - (xx[2] * (origState[15] * origState[16] - origState[13] *
                       origState[14]));
  xx[8] = (xx[14] == 0.0 && xx[8] == 0.0) ? 0.0 : atan2(xx[14], xx[8]);
  xx[7] = - (xx[2] * (xx[5] - xx[6]) * xx[11]);
  xx[13] = (xx[3] + xx[4]) * xx[2] * xx[11];
  xx[7] = (xx[13] == 0.0 && xx[7] == 0.0) ? 0.0 : atan2(xx[13], xx[7]);
  xx[2] = bb[0] ? xx[8] : xx[12] * xx[7];
  xx[3] = bb[0] ? xx[10] : xx[11] * xx[2];
  xx[17] = xx[1];
  xx[18] = xx[1];
  xx[19] = xx[1];
  xx[20] = xx[1];
  xx[21] = xx[3];
  xx[22] = xx[3];
  xx[23] = xx[3];
  xx[1] = xx[17 + (partialType)];
  xx[3] = cos(xx[1]);
  xx[4] = sin(xx[1]);
  xx[1] = sin(xx[16]);
  xx[10] = xx[0];
  xx[11] = xx[2];
  xx[12] = xx[0];
  xx[13] = xx[2];
  xx[14] = xx[0];
  xx[15] = xx[2];
  xx[16] = xx[0];
  xx[0] = xx[10 + (partialType)];
  xx[2] = cos(xx[0]);
  xx[5] = sin(xx[0]);
  xx[0] = xx[3] * xx[5];
  xx[6] = xx[3] * xx[2];
  xx[10] = xx[9] * xx[3];
  xx[11] = - (xx[9] * xx[4]);
  xx[12] = xx[1];
  xx[13] = xx[2] * xx[4] + xx[0] * xx[1];
  xx[14] = xx[6] - xx[1] * xx[5] * xx[4];
  xx[15] = - (xx[9] * xx[5]);
  xx[16] = xx[4] * xx[5] - xx[6] * xx[1];
  xx[17] = xx[0] + xx[2] * xx[1] * xx[4];
  xx[18] = xx[9] * xx[2];
  pm_math_Quaternion_Matrix3x3Ctor_ra(xx + 10, xx + 0);
  state[13] = xx[0];
  state[14] = xx[1];
  state[15] = xx[2];
  state[16] = xx[3];
}

static void projectPartiallyTargetedPos_2_0(const double *origState, int
  partialType, double *state)
{
  boolean_T bb[1];
  int ii[3];
  double xx[24];
  xx[0] = state[21] * state[23];
  xx[1] = state[20] * state[22];
  xx[2] = 2.0;
  xx[3] = (xx[0] + xx[1]) * xx[2];
  xx[4] = fabs(xx[3]) > 1.0 ? atan2(xx[3], 0.0) : asin(xx[3]);
  xx[5] = origState[21] * origState[23];
  xx[6] = origState[20] * origState[22];
  xx[7] = (xx[5] + xx[6]) * xx[2];
  xx[8] = fabs(xx[7]) > 1.0 ? atan2(xx[7], 0.0) : asin(xx[7]);
  xx[9] = xx[4];
  xx[10] = xx[4];
  xx[11] = xx[8];
  xx[12] = xx[8];
  xx[13] = xx[4];
  xx[14] = xx[4];
  xx[15] = xx[8];
  xx[16] = xx[9 + (partialType)];
  xx[9] = cos(xx[16]);
  xx[10] = 0.99999999999999;
  xx[11] = fabs(xx[3]) - xx[10];
  if (xx[11] < 0.0)
    ii[0] = -1;
  else if (xx[11] > 0.0)
    ii[0] = +1;
  else
    ii[0] = 0;
  ii[1] = ii[0];
  if (0 > ii[1])
    ii[1] = 0;
  bb[0] = !(ii[1] == 1);
  xx[3] = state[21] * state[22];
  xx[11] = state[20] * state[23];
  xx[12] = state[20] * state[20];
  xx[13] = 1.0;
  xx[14] = (xx[12] + state[21] * state[21]) * xx[2] - xx[13];
  xx[15] = - (xx[2] * (xx[3] - xx[11]));
  xx[14] = (xx[15] == 0.0 && xx[14] == 0.0) ? 0.0 : atan2(xx[15], xx[14]);
  if (xx[4] < 0.0)
    xx[15] = -1.0;
  else if (xx[4] > 0.0)
    xx[15] = +1.0;
  else
    xx[15] = 0.0;
  xx[4] = (xx[12] + state[23] * state[23]) * xx[2] - xx[13];
  xx[17] = - (xx[2] * (state[22] * state[23] - state[20] * state[21]));
  xx[4] = (xx[17] == 0.0 && xx[4] == 0.0) ? 0.0 : atan2(xx[17], xx[4]);
  xx[12] = 0.5;
  xx[17] = - (xx[2] * (xx[0] - xx[1]) * xx[15]);
  xx[18] = (xx[3] + xx[11]) * xx[2] * xx[15];
  xx[17] = (xx[18] == 0.0 && xx[17] == 0.0) ? 0.0 : atan2(xx[18], xx[17]);
  xx[0] = bb[0] ? xx[4] : xx[12] * xx[17];
  xx[1] = bb[0] ? xx[14] : xx[15] * xx[0];
  xx[3] = fabs(xx[7]) - xx[10];
  if (xx[3] < 0.0)
    ii[0] = -1;
  else if (xx[3] > 0.0)
    ii[0] = +1;
  else
    ii[0] = 0;
  ii[1] = ii[0];
  if (0 > ii[1])
    ii[1] = 0;
  bb[0] = !(ii[1] == 1);
  xx[3] = origState[21] * origState[22];
  xx[4] = origState[20] * origState[23];
  xx[7] = origState[20] * origState[20];
  xx[10] = (xx[7] + origState[21] * origState[21]) * xx[2] - xx[13];
  xx[11] = - (xx[2] * (xx[3] - xx[4]));
  xx[10] = (xx[11] == 0.0 && xx[10] == 0.0) ? 0.0 : atan2(xx[11], xx[10]);
  if (xx[8] < 0.0)
    xx[11] = -1.0;
  else if (xx[8] > 0.0)
    xx[11] = +1.0;
  else
    xx[11] = 0.0;
  xx[8] = (xx[7] + origState[23] * origState[23]) * xx[2] - xx[13];
  xx[14] = - (xx[2] * (origState[22] * origState[23] - origState[20] *
                       origState[21]));
  xx[8] = (xx[14] == 0.0 && xx[8] == 0.0) ? 0.0 : atan2(xx[14], xx[8]);
  xx[7] = - (xx[2] * (xx[5] - xx[6]) * xx[11]);
  xx[13] = (xx[3] + xx[4]) * xx[2] * xx[11];
  xx[7] = (xx[13] == 0.0 && xx[7] == 0.0) ? 0.0 : atan2(xx[13], xx[7]);
  xx[2] = bb[0] ? xx[8] : xx[12] * xx[7];
  xx[3] = bb[0] ? xx[10] : xx[11] * xx[2];
  xx[17] = xx[1];
  xx[18] = xx[1];
  xx[19] = xx[1];
  xx[20] = xx[1];
  xx[21] = xx[3];
  xx[22] = xx[3];
  xx[23] = xx[3];
  xx[1] = xx[17 + (partialType)];
  xx[3] = cos(xx[1]);
  xx[4] = sin(xx[1]);
  xx[1] = sin(xx[16]);
  xx[10] = xx[0];
  xx[11] = xx[2];
  xx[12] = xx[0];
  xx[13] = xx[2];
  xx[14] = xx[0];
  xx[15] = xx[2];
  xx[16] = xx[0];
  xx[0] = xx[10 + (partialType)];
  xx[2] = cos(xx[0]);
  xx[5] = sin(xx[0]);
  xx[0] = xx[3] * xx[5];
  xx[6] = xx[3] * xx[2];
  xx[10] = xx[9] * xx[3];
  xx[11] = - (xx[9] * xx[4]);
  xx[12] = xx[1];
  xx[13] = xx[2] * xx[4] + xx[0] * xx[1];
  xx[14] = xx[6] - xx[1] * xx[5] * xx[4];
  xx[15] = - (xx[9] * xx[5]);
  xx[16] = xx[4] * xx[5] - xx[6] * xx[1];
  xx[17] = xx[0] + xx[2] * xx[1] * xx[4];
  xx[18] = xx[9] * xx[2];
  pm_math_Quaternion_Matrix3x3Ctor_ra(xx + 10, xx + 0);
  state[20] = xx[0];
  state[21] = xx[1];
  state[22] = xx[2];
  state[23] = xx[3];
}

static void projectPartiallyTargetedPos_3_0(const double *origState, int
  partialType, double *state)
{
  boolean_T bb[1];
  int ii[3];
  double xx[24];
  xx[0] = state[28] * state[30];
  xx[1] = state[27] * state[29];
  xx[2] = 2.0;
  xx[3] = (xx[0] + xx[1]) * xx[2];
  xx[4] = fabs(xx[3]) > 1.0 ? atan2(xx[3], 0.0) : asin(xx[3]);
  xx[5] = origState[28] * origState[30];
  xx[6] = origState[27] * origState[29];
  xx[7] = (xx[5] + xx[6]) * xx[2];
  xx[8] = fabs(xx[7]) > 1.0 ? atan2(xx[7], 0.0) : asin(xx[7]);
  xx[9] = xx[4];
  xx[10] = xx[4];
  xx[11] = xx[8];
  xx[12] = xx[8];
  xx[13] = xx[4];
  xx[14] = xx[4];
  xx[15] = xx[8];
  xx[16] = xx[9 + (partialType)];
  xx[9] = cos(xx[16]);
  xx[10] = 0.99999999999999;
  xx[11] = fabs(xx[3]) - xx[10];
  if (xx[11] < 0.0)
    ii[0] = -1;
  else if (xx[11] > 0.0)
    ii[0] = +1;
  else
    ii[0] = 0;
  ii[1] = ii[0];
  if (0 > ii[1])
    ii[1] = 0;
  bb[0] = !(ii[1] == 1);
  xx[3] = state[28] * state[29];
  xx[11] = state[27] * state[30];
  xx[12] = state[27] * state[27];
  xx[13] = 1.0;
  xx[14] = (xx[12] + state[28] * state[28]) * xx[2] - xx[13];
  xx[15] = - (xx[2] * (xx[3] - xx[11]));
  xx[14] = (xx[15] == 0.0 && xx[14] == 0.0) ? 0.0 : atan2(xx[15], xx[14]);
  if (xx[4] < 0.0)
    xx[15] = -1.0;
  else if (xx[4] > 0.0)
    xx[15] = +1.0;
  else
    xx[15] = 0.0;
  xx[4] = (xx[12] + state[30] * state[30]) * xx[2] - xx[13];
  xx[17] = - (xx[2] * (state[29] * state[30] - state[27] * state[28]));
  xx[4] = (xx[17] == 0.0 && xx[4] == 0.0) ? 0.0 : atan2(xx[17], xx[4]);
  xx[12] = 0.5;
  xx[17] = - (xx[2] * (xx[0] - xx[1]) * xx[15]);
  xx[18] = (xx[3] + xx[11]) * xx[2] * xx[15];
  xx[17] = (xx[18] == 0.0 && xx[17] == 0.0) ? 0.0 : atan2(xx[18], xx[17]);
  xx[0] = bb[0] ? xx[4] : xx[12] * xx[17];
  xx[1] = bb[0] ? xx[14] : xx[15] * xx[0];
  xx[3] = fabs(xx[7]) - xx[10];
  if (xx[3] < 0.0)
    ii[0] = -1;
  else if (xx[3] > 0.0)
    ii[0] = +1;
  else
    ii[0] = 0;
  ii[1] = ii[0];
  if (0 > ii[1])
    ii[1] = 0;
  bb[0] = !(ii[1] == 1);
  xx[3] = origState[28] * origState[29];
  xx[4] = origState[27] * origState[30];
  xx[7] = origState[27] * origState[27];
  xx[10] = (xx[7] + origState[28] * origState[28]) * xx[2] - xx[13];
  xx[11] = - (xx[2] * (xx[3] - xx[4]));
  xx[10] = (xx[11] == 0.0 && xx[10] == 0.0) ? 0.0 : atan2(xx[11], xx[10]);
  if (xx[8] < 0.0)
    xx[11] = -1.0;
  else if (xx[8] > 0.0)
    xx[11] = +1.0;
  else
    xx[11] = 0.0;
  xx[8] = (xx[7] + origState[30] * origState[30]) * xx[2] - xx[13];
  xx[14] = - (xx[2] * (origState[29] * origState[30] - origState[27] *
                       origState[28]));
  xx[8] = (xx[14] == 0.0 && xx[8] == 0.0) ? 0.0 : atan2(xx[14], xx[8]);
  xx[7] = - (xx[2] * (xx[5] - xx[6]) * xx[11]);
  xx[13] = (xx[3] + xx[4]) * xx[2] * xx[11];
  xx[7] = (xx[13] == 0.0 && xx[7] == 0.0) ? 0.0 : atan2(xx[13], xx[7]);
  xx[2] = bb[0] ? xx[8] : xx[12] * xx[7];
  xx[3] = bb[0] ? xx[10] : xx[11] * xx[2];
  xx[17] = xx[1];
  xx[18] = xx[1];
  xx[19] = xx[1];
  xx[20] = xx[1];
  xx[21] = xx[3];
  xx[22] = xx[3];
  xx[23] = xx[3];
  xx[1] = xx[17 + (partialType)];
  xx[3] = cos(xx[1]);
  xx[4] = sin(xx[1]);
  xx[1] = sin(xx[16]);
  xx[10] = xx[0];
  xx[11] = xx[2];
  xx[12] = xx[0];
  xx[13] = xx[2];
  xx[14] = xx[0];
  xx[15] = xx[2];
  xx[16] = xx[0];
  xx[0] = xx[10 + (partialType)];
  xx[2] = cos(xx[0]);
  xx[5] = sin(xx[0]);
  xx[0] = xx[3] * xx[5];
  xx[6] = xx[3] * xx[2];
  xx[10] = xx[9] * xx[3];
  xx[11] = - (xx[9] * xx[4]);
  xx[12] = xx[1];
  xx[13] = xx[2] * xx[4] + xx[0] * xx[1];
  xx[14] = xx[6] - xx[1] * xx[5] * xx[4];
  xx[15] = - (xx[9] * xx[5]);
  xx[16] = xx[4] * xx[5] - xx[6] * xx[1];
  xx[17] = xx[0] + xx[2] * xx[1] * xx[4];
  xx[18] = xx[9] * xx[2];
  pm_math_Quaternion_Matrix3x3Ctor_ra(xx + 10, xx + 0);
  state[27] = xx[0];
  state[28] = xx[1];
  state[29] = xx[2];
  state[30] = xx[3];
}

static void projectPartiallyTargetedPos_6_0(const double *origState, int
  partialType, double *state)
{
  boolean_T bb[1];
  int ii[3];
  double xx[24];
  xx[0] = state[39] * state[41];
  xx[1] = state[38] * state[40];
  xx[2] = 2.0;
  xx[3] = (xx[0] + xx[1]) * xx[2];
  xx[4] = fabs(xx[3]) > 1.0 ? atan2(xx[3], 0.0) : asin(xx[3]);
  xx[5] = origState[39] * origState[41];
  xx[6] = origState[38] * origState[40];
  xx[7] = (xx[5] + xx[6]) * xx[2];
  xx[8] = fabs(xx[7]) > 1.0 ? atan2(xx[7], 0.0) : asin(xx[7]);
  xx[9] = xx[4];
  xx[10] = xx[4];
  xx[11] = xx[8];
  xx[12] = xx[8];
  xx[13] = xx[4];
  xx[14] = xx[4];
  xx[15] = xx[8];
  xx[16] = xx[9 + (partialType)];
  xx[9] = cos(xx[16]);
  xx[10] = 0.99999999999999;
  xx[11] = fabs(xx[3]) - xx[10];
  if (xx[11] < 0.0)
    ii[0] = -1;
  else if (xx[11] > 0.0)
    ii[0] = +1;
  else
    ii[0] = 0;
  ii[1] = ii[0];
  if (0 > ii[1])
    ii[1] = 0;
  bb[0] = !(ii[1] == 1);
  xx[3] = state[39] * state[40];
  xx[11] = state[38] * state[41];
  xx[12] = state[38] * state[38];
  xx[13] = 1.0;
  xx[14] = (xx[12] + state[39] * state[39]) * xx[2] - xx[13];
  xx[15] = - (xx[2] * (xx[3] - xx[11]));
  xx[14] = (xx[15] == 0.0 && xx[14] == 0.0) ? 0.0 : atan2(xx[15], xx[14]);
  if (xx[4] < 0.0)
    xx[15] = -1.0;
  else if (xx[4] > 0.0)
    xx[15] = +1.0;
  else
    xx[15] = 0.0;
  xx[4] = (xx[12] + state[41] * state[41]) * xx[2] - xx[13];
  xx[17] = - (xx[2] * (state[40] * state[41] - state[38] * state[39]));
  xx[4] = (xx[17] == 0.0 && xx[4] == 0.0) ? 0.0 : atan2(xx[17], xx[4]);
  xx[12] = 0.5;
  xx[17] = - (xx[2] * (xx[0] - xx[1]) * xx[15]);
  xx[18] = (xx[3] + xx[11]) * xx[2] * xx[15];
  xx[17] = (xx[18] == 0.0 && xx[17] == 0.0) ? 0.0 : atan2(xx[18], xx[17]);
  xx[0] = bb[0] ? xx[4] : xx[12] * xx[17];
  xx[1] = bb[0] ? xx[14] : xx[15] * xx[0];
  xx[3] = fabs(xx[7]) - xx[10];
  if (xx[3] < 0.0)
    ii[0] = -1;
  else if (xx[3] > 0.0)
    ii[0] = +1;
  else
    ii[0] = 0;
  ii[1] = ii[0];
  if (0 > ii[1])
    ii[1] = 0;
  bb[0] = !(ii[1] == 1);
  xx[3] = origState[39] * origState[40];
  xx[4] = origState[38] * origState[41];
  xx[7] = origState[38] * origState[38];
  xx[10] = (xx[7] + origState[39] * origState[39]) * xx[2] - xx[13];
  xx[11] = - (xx[2] * (xx[3] - xx[4]));
  xx[10] = (xx[11] == 0.0 && xx[10] == 0.0) ? 0.0 : atan2(xx[11], xx[10]);
  if (xx[8] < 0.0)
    xx[11] = -1.0;
  else if (xx[8] > 0.0)
    xx[11] = +1.0;
  else
    xx[11] = 0.0;
  xx[8] = (xx[7] + origState[41] * origState[41]) * xx[2] - xx[13];
  xx[14] = - (xx[2] * (origState[40] * origState[41] - origState[38] *
                       origState[39]));
  xx[8] = (xx[14] == 0.0 && xx[8] == 0.0) ? 0.0 : atan2(xx[14], xx[8]);
  xx[7] = - (xx[2] * (xx[5] - xx[6]) * xx[11]);
  xx[13] = (xx[3] + xx[4]) * xx[2] * xx[11];
  xx[7] = (xx[13] == 0.0 && xx[7] == 0.0) ? 0.0 : atan2(xx[13], xx[7]);
  xx[2] = bb[0] ? xx[8] : xx[12] * xx[7];
  xx[3] = bb[0] ? xx[10] : xx[11] * xx[2];
  xx[17] = xx[1];
  xx[18] = xx[1];
  xx[19] = xx[1];
  xx[20] = xx[1];
  xx[21] = xx[3];
  xx[22] = xx[3];
  xx[23] = xx[3];
  xx[1] = xx[17 + (partialType)];
  xx[3] = cos(xx[1]);
  xx[4] = sin(xx[1]);
  xx[1] = sin(xx[16]);
  xx[10] = xx[0];
  xx[11] = xx[2];
  xx[12] = xx[0];
  xx[13] = xx[2];
  xx[14] = xx[0];
  xx[15] = xx[2];
  xx[16] = xx[0];
  xx[0] = xx[10 + (partialType)];
  xx[2] = cos(xx[0]);
  xx[5] = sin(xx[0]);
  xx[0] = xx[3] * xx[5];
  xx[6] = xx[3] * xx[2];
  xx[10] = xx[9] * xx[3];
  xx[11] = - (xx[9] * xx[4]);
  xx[12] = xx[1];
  xx[13] = xx[2] * xx[4] + xx[0] * xx[1];
  xx[14] = xx[6] - xx[1] * xx[5] * xx[4];
  xx[15] = - (xx[9] * xx[5]);
  xx[16] = xx[4] * xx[5] - xx[6] * xx[1];
  xx[17] = xx[0] + xx[2] * xx[1] * xx[4];
  xx[18] = xx[9] * xx[2];
  pm_math_Quaternion_Matrix3x3Ctor_ra(xx + 10, xx + 0);
  state[38] = xx[0];
  state[39] = xx[1];
  state[40] = xx[2];
  state[41] = xx[3];
}

static void projectPartiallyTargetedPos_7_0(const double *origState, int
  partialType, double *state)
{
  boolean_T bb[1];
  int ii[3];
  double xx[24];
  xx[0] = state[46] * state[48];
  xx[1] = state[45] * state[47];
  xx[2] = 2.0;
  xx[3] = (xx[0] + xx[1]) * xx[2];
  xx[4] = fabs(xx[3]) > 1.0 ? atan2(xx[3], 0.0) : asin(xx[3]);
  xx[5] = origState[46] * origState[48];
  xx[6] = origState[45] * origState[47];
  xx[7] = (xx[5] + xx[6]) * xx[2];
  xx[8] = fabs(xx[7]) > 1.0 ? atan2(xx[7], 0.0) : asin(xx[7]);
  xx[9] = xx[4];
  xx[10] = xx[4];
  xx[11] = xx[8];
  xx[12] = xx[8];
  xx[13] = xx[4];
  xx[14] = xx[4];
  xx[15] = xx[8];
  xx[16] = xx[9 + (partialType)];
  xx[9] = cos(xx[16]);
  xx[10] = 0.99999999999999;
  xx[11] = fabs(xx[3]) - xx[10];
  if (xx[11] < 0.0)
    ii[0] = -1;
  else if (xx[11] > 0.0)
    ii[0] = +1;
  else
    ii[0] = 0;
  ii[1] = ii[0];
  if (0 > ii[1])
    ii[1] = 0;
  bb[0] = !(ii[1] == 1);
  xx[3] = state[46] * state[47];
  xx[11] = state[45] * state[48];
  xx[12] = state[45] * state[45];
  xx[13] = 1.0;
  xx[14] = (xx[12] + state[46] * state[46]) * xx[2] - xx[13];
  xx[15] = - (xx[2] * (xx[3] - xx[11]));
  xx[14] = (xx[15] == 0.0 && xx[14] == 0.0) ? 0.0 : atan2(xx[15], xx[14]);
  if (xx[4] < 0.0)
    xx[15] = -1.0;
  else if (xx[4] > 0.0)
    xx[15] = +1.0;
  else
    xx[15] = 0.0;
  xx[4] = (xx[12] + state[48] * state[48]) * xx[2] - xx[13];
  xx[17] = - (xx[2] * (state[47] * state[48] - state[45] * state[46]));
  xx[4] = (xx[17] == 0.0 && xx[4] == 0.0) ? 0.0 : atan2(xx[17], xx[4]);
  xx[12] = 0.5;
  xx[17] = - (xx[2] * (xx[0] - xx[1]) * xx[15]);
  xx[18] = (xx[3] + xx[11]) * xx[2] * xx[15];
  xx[17] = (xx[18] == 0.0 && xx[17] == 0.0) ? 0.0 : atan2(xx[18], xx[17]);
  xx[0] = bb[0] ? xx[4] : xx[12] * xx[17];
  xx[1] = bb[0] ? xx[14] : xx[15] * xx[0];
  xx[3] = fabs(xx[7]) - xx[10];
  if (xx[3] < 0.0)
    ii[0] = -1;
  else if (xx[3] > 0.0)
    ii[0] = +1;
  else
    ii[0] = 0;
  ii[1] = ii[0];
  if (0 > ii[1])
    ii[1] = 0;
  bb[0] = !(ii[1] == 1);
  xx[3] = origState[46] * origState[47];
  xx[4] = origState[45] * origState[48];
  xx[7] = origState[45] * origState[45];
  xx[10] = (xx[7] + origState[46] * origState[46]) * xx[2] - xx[13];
  xx[11] = - (xx[2] * (xx[3] - xx[4]));
  xx[10] = (xx[11] == 0.0 && xx[10] == 0.0) ? 0.0 : atan2(xx[11], xx[10]);
  if (xx[8] < 0.0)
    xx[11] = -1.0;
  else if (xx[8] > 0.0)
    xx[11] = +1.0;
  else
    xx[11] = 0.0;
  xx[8] = (xx[7] + origState[48] * origState[48]) * xx[2] - xx[13];
  xx[14] = - (xx[2] * (origState[47] * origState[48] - origState[45] *
                       origState[46]));
  xx[8] = (xx[14] == 0.0 && xx[8] == 0.0) ? 0.0 : atan2(xx[14], xx[8]);
  xx[7] = - (xx[2] * (xx[5] - xx[6]) * xx[11]);
  xx[13] = (xx[3] + xx[4]) * xx[2] * xx[11];
  xx[7] = (xx[13] == 0.0 && xx[7] == 0.0) ? 0.0 : atan2(xx[13], xx[7]);
  xx[2] = bb[0] ? xx[8] : xx[12] * xx[7];
  xx[3] = bb[0] ? xx[10] : xx[11] * xx[2];
  xx[17] = xx[1];
  xx[18] = xx[1];
  xx[19] = xx[1];
  xx[20] = xx[1];
  xx[21] = xx[3];
  xx[22] = xx[3];
  xx[23] = xx[3];
  xx[1] = xx[17 + (partialType)];
  xx[3] = cos(xx[1]);
  xx[4] = sin(xx[1]);
  xx[1] = sin(xx[16]);
  xx[10] = xx[0];
  xx[11] = xx[2];
  xx[12] = xx[0];
  xx[13] = xx[2];
  xx[14] = xx[0];
  xx[15] = xx[2];
  xx[16] = xx[0];
  xx[0] = xx[10 + (partialType)];
  xx[2] = cos(xx[0]);
  xx[5] = sin(xx[0]);
  xx[0] = xx[3] * xx[5];
  xx[6] = xx[3] * xx[2];
  xx[10] = xx[9] * xx[3];
  xx[11] = - (xx[9] * xx[4]);
  xx[12] = xx[1];
  xx[13] = xx[2] * xx[4] + xx[0] * xx[1];
  xx[14] = xx[6] - xx[1] * xx[5] * xx[4];
  xx[15] = - (xx[9] * xx[5]);
  xx[16] = xx[4] * xx[5] - xx[6] * xx[1];
  xx[17] = xx[0] + xx[2] * xx[1] * xx[4];
  xx[18] = xx[9] * xx[2];
  pm_math_Quaternion_Matrix3x3Ctor_ra(xx + 10, xx + 0);
  state[45] = xx[0];
  state[46] = xx[1];
  state[47] = xx[2];
  state[48] = xx[3];
}

static void projectPartiallyTargetedPos_8_0(const double *origState, int
  partialType, double *state)
{
  boolean_T bb[1];
  int ii[3];
  double xx[24];
  xx[0] = state[53] * state[55];
  xx[1] = state[52] * state[54];
  xx[2] = 2.0;
  xx[3] = (xx[0] + xx[1]) * xx[2];
  xx[4] = fabs(xx[3]) > 1.0 ? atan2(xx[3], 0.0) : asin(xx[3]);
  xx[5] = origState[53] * origState[55];
  xx[6] = origState[52] * origState[54];
  xx[7] = (xx[5] + xx[6]) * xx[2];
  xx[8] = fabs(xx[7]) > 1.0 ? atan2(xx[7], 0.0) : asin(xx[7]);
  xx[9] = xx[4];
  xx[10] = xx[4];
  xx[11] = xx[8];
  xx[12] = xx[8];
  xx[13] = xx[4];
  xx[14] = xx[4];
  xx[15] = xx[8];
  xx[16] = xx[9 + (partialType)];
  xx[9] = cos(xx[16]);
  xx[10] = 0.99999999999999;
  xx[11] = fabs(xx[3]) - xx[10];
  if (xx[11] < 0.0)
    ii[0] = -1;
  else if (xx[11] > 0.0)
    ii[0] = +1;
  else
    ii[0] = 0;
  ii[1] = ii[0];
  if (0 > ii[1])
    ii[1] = 0;
  bb[0] = !(ii[1] == 1);
  xx[3] = state[53] * state[54];
  xx[11] = state[52] * state[55];
  xx[12] = state[52] * state[52];
  xx[13] = 1.0;
  xx[14] = (xx[12] + state[53] * state[53]) * xx[2] - xx[13];
  xx[15] = - (xx[2] * (xx[3] - xx[11]));
  xx[14] = (xx[15] == 0.0 && xx[14] == 0.0) ? 0.0 : atan2(xx[15], xx[14]);
  if (xx[4] < 0.0)
    xx[15] = -1.0;
  else if (xx[4] > 0.0)
    xx[15] = +1.0;
  else
    xx[15] = 0.0;
  xx[4] = (xx[12] + state[55] * state[55]) * xx[2] - xx[13];
  xx[17] = - (xx[2] * (state[54] * state[55] - state[52] * state[53]));
  xx[4] = (xx[17] == 0.0 && xx[4] == 0.0) ? 0.0 : atan2(xx[17], xx[4]);
  xx[12] = 0.5;
  xx[17] = - (xx[2] * (xx[0] - xx[1]) * xx[15]);
  xx[18] = (xx[3] + xx[11]) * xx[2] * xx[15];
  xx[17] = (xx[18] == 0.0 && xx[17] == 0.0) ? 0.0 : atan2(xx[18], xx[17]);
  xx[0] = bb[0] ? xx[4] : xx[12] * xx[17];
  xx[1] = bb[0] ? xx[14] : xx[15] * xx[0];
  xx[3] = fabs(xx[7]) - xx[10];
  if (xx[3] < 0.0)
    ii[0] = -1;
  else if (xx[3] > 0.0)
    ii[0] = +1;
  else
    ii[0] = 0;
  ii[1] = ii[0];
  if (0 > ii[1])
    ii[1] = 0;
  bb[0] = !(ii[1] == 1);
  xx[3] = origState[53] * origState[54];
  xx[4] = origState[52] * origState[55];
  xx[7] = origState[52] * origState[52];
  xx[10] = (xx[7] + origState[53] * origState[53]) * xx[2] - xx[13];
  xx[11] = - (xx[2] * (xx[3] - xx[4]));
  xx[10] = (xx[11] == 0.0 && xx[10] == 0.0) ? 0.0 : atan2(xx[11], xx[10]);
  if (xx[8] < 0.0)
    xx[11] = -1.0;
  else if (xx[8] > 0.0)
    xx[11] = +1.0;
  else
    xx[11] = 0.0;
  xx[8] = (xx[7] + origState[55] * origState[55]) * xx[2] - xx[13];
  xx[14] = - (xx[2] * (origState[54] * origState[55] - origState[52] *
                       origState[53]));
  xx[8] = (xx[14] == 0.0 && xx[8] == 0.0) ? 0.0 : atan2(xx[14], xx[8]);
  xx[7] = - (xx[2] * (xx[5] - xx[6]) * xx[11]);
  xx[13] = (xx[3] + xx[4]) * xx[2] * xx[11];
  xx[7] = (xx[13] == 0.0 && xx[7] == 0.0) ? 0.0 : atan2(xx[13], xx[7]);
  xx[2] = bb[0] ? xx[8] : xx[12] * xx[7];
  xx[3] = bb[0] ? xx[10] : xx[11] * xx[2];
  xx[17] = xx[1];
  xx[18] = xx[1];
  xx[19] = xx[1];
  xx[20] = xx[1];
  xx[21] = xx[3];
  xx[22] = xx[3];
  xx[23] = xx[3];
  xx[1] = xx[17 + (partialType)];
  xx[3] = cos(xx[1]);
  xx[4] = sin(xx[1]);
  xx[1] = sin(xx[16]);
  xx[10] = xx[0];
  xx[11] = xx[2];
  xx[12] = xx[0];
  xx[13] = xx[2];
  xx[14] = xx[0];
  xx[15] = xx[2];
  xx[16] = xx[0];
  xx[0] = xx[10 + (partialType)];
  xx[2] = cos(xx[0]);
  xx[5] = sin(xx[0]);
  xx[0] = xx[3] * xx[5];
  xx[6] = xx[3] * xx[2];
  xx[10] = xx[9] * xx[3];
  xx[11] = - (xx[9] * xx[4]);
  xx[12] = xx[1];
  xx[13] = xx[2] * xx[4] + xx[0] * xx[1];
  xx[14] = xx[6] - xx[1] * xx[5] * xx[4];
  xx[15] = - (xx[9] * xx[5]);
  xx[16] = xx[4] * xx[5] - xx[6] * xx[1];
  xx[17] = xx[0] + xx[2] * xx[1] * xx[4];
  xx[18] = xx[9] * xx[2];
  pm_math_Quaternion_Matrix3x3Ctor_ra(xx + 10, xx + 0);
  state[52] = xx[0];
  state[53] = xx[1];
  state[54] = xx[2];
  state[55] = xx[3];
}

static void projectPartiallyTargetedPos_9_0(const double *origState, int
  partialType, double *state)
{
  boolean_T bb[1];
  int ii[3];
  double xx[24];
  xx[0] = state[60] * state[62];
  xx[1] = state[59] * state[61];
  xx[2] = 2.0;
  xx[3] = (xx[0] + xx[1]) * xx[2];
  xx[4] = fabs(xx[3]) > 1.0 ? atan2(xx[3], 0.0) : asin(xx[3]);
  xx[5] = origState[60] * origState[62];
  xx[6] = origState[59] * origState[61];
  xx[7] = (xx[5] + xx[6]) * xx[2];
  xx[8] = fabs(xx[7]) > 1.0 ? atan2(xx[7], 0.0) : asin(xx[7]);
  xx[9] = xx[4];
  xx[10] = xx[4];
  xx[11] = xx[8];
  xx[12] = xx[8];
  xx[13] = xx[4];
  xx[14] = xx[4];
  xx[15] = xx[8];
  xx[16] = xx[9 + (partialType)];
  xx[9] = cos(xx[16]);
  xx[10] = 0.99999999999999;
  xx[11] = fabs(xx[3]) - xx[10];
  if (xx[11] < 0.0)
    ii[0] = -1;
  else if (xx[11] > 0.0)
    ii[0] = +1;
  else
    ii[0] = 0;
  ii[1] = ii[0];
  if (0 > ii[1])
    ii[1] = 0;
  bb[0] = !(ii[1] == 1);
  xx[3] = state[60] * state[61];
  xx[11] = state[59] * state[62];
  xx[12] = state[59] * state[59];
  xx[13] = 1.0;
  xx[14] = (xx[12] + state[60] * state[60]) * xx[2] - xx[13];
  xx[15] = - (xx[2] * (xx[3] - xx[11]));
  xx[14] = (xx[15] == 0.0 && xx[14] == 0.0) ? 0.0 : atan2(xx[15], xx[14]);
  if (xx[4] < 0.0)
    xx[15] = -1.0;
  else if (xx[4] > 0.0)
    xx[15] = +1.0;
  else
    xx[15] = 0.0;
  xx[4] = (xx[12] + state[62] * state[62]) * xx[2] - xx[13];
  xx[17] = - (xx[2] * (state[61] * state[62] - state[59] * state[60]));
  xx[4] = (xx[17] == 0.0 && xx[4] == 0.0) ? 0.0 : atan2(xx[17], xx[4]);
  xx[12] = 0.5;
  xx[17] = - (xx[2] * (xx[0] - xx[1]) * xx[15]);
  xx[18] = (xx[3] + xx[11]) * xx[2] * xx[15];
  xx[17] = (xx[18] == 0.0 && xx[17] == 0.0) ? 0.0 : atan2(xx[18], xx[17]);
  xx[0] = bb[0] ? xx[4] : xx[12] * xx[17];
  xx[1] = bb[0] ? xx[14] : xx[15] * xx[0];
  xx[3] = fabs(xx[7]) - xx[10];
  if (xx[3] < 0.0)
    ii[0] = -1;
  else if (xx[3] > 0.0)
    ii[0] = +1;
  else
    ii[0] = 0;
  ii[1] = ii[0];
  if (0 > ii[1])
    ii[1] = 0;
  bb[0] = !(ii[1] == 1);
  xx[3] = origState[60] * origState[61];
  xx[4] = origState[59] * origState[62];
  xx[7] = origState[59] * origState[59];
  xx[10] = (xx[7] + origState[60] * origState[60]) * xx[2] - xx[13];
  xx[11] = - (xx[2] * (xx[3] - xx[4]));
  xx[10] = (xx[11] == 0.0 && xx[10] == 0.0) ? 0.0 : atan2(xx[11], xx[10]);
  if (xx[8] < 0.0)
    xx[11] = -1.0;
  else if (xx[8] > 0.0)
    xx[11] = +1.0;
  else
    xx[11] = 0.0;
  xx[8] = (xx[7] + origState[62] * origState[62]) * xx[2] - xx[13];
  xx[14] = - (xx[2] * (origState[61] * origState[62] - origState[59] *
                       origState[60]));
  xx[8] = (xx[14] == 0.0 && xx[8] == 0.0) ? 0.0 : atan2(xx[14], xx[8]);
  xx[7] = - (xx[2] * (xx[5] - xx[6]) * xx[11]);
  xx[13] = (xx[3] + xx[4]) * xx[2] * xx[11];
  xx[7] = (xx[13] == 0.0 && xx[7] == 0.0) ? 0.0 : atan2(xx[13], xx[7]);
  xx[2] = bb[0] ? xx[8] : xx[12] * xx[7];
  xx[3] = bb[0] ? xx[10] : xx[11] * xx[2];
  xx[17] = xx[1];
  xx[18] = xx[1];
  xx[19] = xx[1];
  xx[20] = xx[1];
  xx[21] = xx[3];
  xx[22] = xx[3];
  xx[23] = xx[3];
  xx[1] = xx[17 + (partialType)];
  xx[3] = cos(xx[1]);
  xx[4] = sin(xx[1]);
  xx[1] = sin(xx[16]);
  xx[10] = xx[0];
  xx[11] = xx[2];
  xx[12] = xx[0];
  xx[13] = xx[2];
  xx[14] = xx[0];
  xx[15] = xx[2];
  xx[16] = xx[0];
  xx[0] = xx[10 + (partialType)];
  xx[2] = cos(xx[0]);
  xx[5] = sin(xx[0]);
  xx[0] = xx[3] * xx[5];
  xx[6] = xx[3] * xx[2];
  xx[10] = xx[9] * xx[3];
  xx[11] = - (xx[9] * xx[4]);
  xx[12] = xx[1];
  xx[13] = xx[2] * xx[4] + xx[0] * xx[1];
  xx[14] = xx[6] - xx[1] * xx[5] * xx[4];
  xx[15] = - (xx[9] * xx[5]);
  xx[16] = xx[4] * xx[5] - xx[6] * xx[1];
  xx[17] = xx[0] + xx[2] * xx[1] * xx[4];
  xx[18] = xx[9] * xx[2];
  pm_math_Quaternion_Matrix3x3Ctor_ra(xx + 10, xx + 0);
  state[59] = xx[0];
  state[60] = xx[1];
  state[61] = xx[2];
  state[62] = xx[3];
}

static void projectPartiallyTargetedPos_10_0(const double *origState, int
  partialType, double *state)
{
  boolean_T bb[1];
  int ii[3];
  double xx[24];
  xx[0] = state[67] * state[69];
  xx[1] = state[66] * state[68];
  xx[2] = 2.0;
  xx[3] = (xx[0] + xx[1]) * xx[2];
  xx[4] = fabs(xx[3]) > 1.0 ? atan2(xx[3], 0.0) : asin(xx[3]);
  xx[5] = origState[67] * origState[69];
  xx[6] = origState[66] * origState[68];
  xx[7] = (xx[5] + xx[6]) * xx[2];
  xx[8] = fabs(xx[7]) > 1.0 ? atan2(xx[7], 0.0) : asin(xx[7]);
  xx[9] = xx[4];
  xx[10] = xx[4];
  xx[11] = xx[8];
  xx[12] = xx[8];
  xx[13] = xx[4];
  xx[14] = xx[4];
  xx[15] = xx[8];
  xx[16] = xx[9 + (partialType)];
  xx[9] = cos(xx[16]);
  xx[10] = 0.99999999999999;
  xx[11] = fabs(xx[3]) - xx[10];
  if (xx[11] < 0.0)
    ii[0] = -1;
  else if (xx[11] > 0.0)
    ii[0] = +1;
  else
    ii[0] = 0;
  ii[1] = ii[0];
  if (0 > ii[1])
    ii[1] = 0;
  bb[0] = !(ii[1] == 1);
  xx[3] = state[67] * state[68];
  xx[11] = state[66] * state[69];
  xx[12] = state[66] * state[66];
  xx[13] = 1.0;
  xx[14] = (xx[12] + state[67] * state[67]) * xx[2] - xx[13];
  xx[15] = - (xx[2] * (xx[3] - xx[11]));
  xx[14] = (xx[15] == 0.0 && xx[14] == 0.0) ? 0.0 : atan2(xx[15], xx[14]);
  if (xx[4] < 0.0)
    xx[15] = -1.0;
  else if (xx[4] > 0.0)
    xx[15] = +1.0;
  else
    xx[15] = 0.0;
  xx[4] = (xx[12] + state[69] * state[69]) * xx[2] - xx[13];
  xx[17] = - (xx[2] * (state[68] * state[69] - state[66] * state[67]));
  xx[4] = (xx[17] == 0.0 && xx[4] == 0.0) ? 0.0 : atan2(xx[17], xx[4]);
  xx[12] = 0.5;
  xx[17] = - (xx[2] * (xx[0] - xx[1]) * xx[15]);
  xx[18] = (xx[3] + xx[11]) * xx[2] * xx[15];
  xx[17] = (xx[18] == 0.0 && xx[17] == 0.0) ? 0.0 : atan2(xx[18], xx[17]);
  xx[0] = bb[0] ? xx[4] : xx[12] * xx[17];
  xx[1] = bb[0] ? xx[14] : xx[15] * xx[0];
  xx[3] = fabs(xx[7]) - xx[10];
  if (xx[3] < 0.0)
    ii[0] = -1;
  else if (xx[3] > 0.0)
    ii[0] = +1;
  else
    ii[0] = 0;
  ii[1] = ii[0];
  if (0 > ii[1])
    ii[1] = 0;
  bb[0] = !(ii[1] == 1);
  xx[3] = origState[67] * origState[68];
  xx[4] = origState[66] * origState[69];
  xx[7] = origState[66] * origState[66];
  xx[10] = (xx[7] + origState[67] * origState[67]) * xx[2] - xx[13];
  xx[11] = - (xx[2] * (xx[3] - xx[4]));
  xx[10] = (xx[11] == 0.0 && xx[10] == 0.0) ? 0.0 : atan2(xx[11], xx[10]);
  if (xx[8] < 0.0)
    xx[11] = -1.0;
  else if (xx[8] > 0.0)
    xx[11] = +1.0;
  else
    xx[11] = 0.0;
  xx[8] = (xx[7] + origState[69] * origState[69]) * xx[2] - xx[13];
  xx[14] = - (xx[2] * (origState[68] * origState[69] - origState[66] *
                       origState[67]));
  xx[8] = (xx[14] == 0.0 && xx[8] == 0.0) ? 0.0 : atan2(xx[14], xx[8]);
  xx[7] = - (xx[2] * (xx[5] - xx[6]) * xx[11]);
  xx[13] = (xx[3] + xx[4]) * xx[2] * xx[11];
  xx[7] = (xx[13] == 0.0 && xx[7] == 0.0) ? 0.0 : atan2(xx[13], xx[7]);
  xx[2] = bb[0] ? xx[8] : xx[12] * xx[7];
  xx[3] = bb[0] ? xx[10] : xx[11] * xx[2];
  xx[17] = xx[1];
  xx[18] = xx[1];
  xx[19] = xx[1];
  xx[20] = xx[1];
  xx[21] = xx[3];
  xx[22] = xx[3];
  xx[23] = xx[3];
  xx[1] = xx[17 + (partialType)];
  xx[3] = cos(xx[1]);
  xx[4] = sin(xx[1]);
  xx[1] = sin(xx[16]);
  xx[10] = xx[0];
  xx[11] = xx[2];
  xx[12] = xx[0];
  xx[13] = xx[2];
  xx[14] = xx[0];
  xx[15] = xx[2];
  xx[16] = xx[0];
  xx[0] = xx[10 + (partialType)];
  xx[2] = cos(xx[0]);
  xx[5] = sin(xx[0]);
  xx[0] = xx[3] * xx[5];
  xx[6] = xx[3] * xx[2];
  xx[10] = xx[9] * xx[3];
  xx[11] = - (xx[9] * xx[4]);
  xx[12] = xx[1];
  xx[13] = xx[2] * xx[4] + xx[0] * xx[1];
  xx[14] = xx[6] - xx[1] * xx[5] * xx[4];
  xx[15] = - (xx[9] * xx[5]);
  xx[16] = xx[4] * xx[5] - xx[6] * xx[1];
  xx[17] = xx[0] + xx[2] * xx[1] * xx[4];
  xx[18] = xx[9] * xx[2];
  pm_math_Quaternion_Matrix3x3Ctor_ra(xx + 10, xx + 0);
  state[66] = xx[0];
  state[67] = xx[1];
  state[68] = xx[2];
  state[69] = xx[3];
}

static void projectPartiallyTargetedPos_11_0(const double *origState, int
  partialType, double *state)
{
  boolean_T bb[1];
  int ii[3];
  double xx[24];
  xx[0] = state[74] * state[76];
  xx[1] = state[73] * state[75];
  xx[2] = 2.0;
  xx[3] = (xx[0] + xx[1]) * xx[2];
  xx[4] = fabs(xx[3]) > 1.0 ? atan2(xx[3], 0.0) : asin(xx[3]);
  xx[5] = origState[74] * origState[76];
  xx[6] = origState[73] * origState[75];
  xx[7] = (xx[5] + xx[6]) * xx[2];
  xx[8] = fabs(xx[7]) > 1.0 ? atan2(xx[7], 0.0) : asin(xx[7]);
  xx[9] = xx[4];
  xx[10] = xx[4];
  xx[11] = xx[8];
  xx[12] = xx[8];
  xx[13] = xx[4];
  xx[14] = xx[4];
  xx[15] = xx[8];
  xx[16] = xx[9 + (partialType)];
  xx[9] = cos(xx[16]);
  xx[10] = 0.99999999999999;
  xx[11] = fabs(xx[3]) - xx[10];
  if (xx[11] < 0.0)
    ii[0] = -1;
  else if (xx[11] > 0.0)
    ii[0] = +1;
  else
    ii[0] = 0;
  ii[1] = ii[0];
  if (0 > ii[1])
    ii[1] = 0;
  bb[0] = !(ii[1] == 1);
  xx[3] = state[74] * state[75];
  xx[11] = state[73] * state[76];
  xx[12] = state[73] * state[73];
  xx[13] = 1.0;
  xx[14] = (xx[12] + state[74] * state[74]) * xx[2] - xx[13];
  xx[15] = - (xx[2] * (xx[3] - xx[11]));
  xx[14] = (xx[15] == 0.0 && xx[14] == 0.0) ? 0.0 : atan2(xx[15], xx[14]);
  if (xx[4] < 0.0)
    xx[15] = -1.0;
  else if (xx[4] > 0.0)
    xx[15] = +1.0;
  else
    xx[15] = 0.0;
  xx[4] = (xx[12] + state[76] * state[76]) * xx[2] - xx[13];
  xx[17] = - (xx[2] * (state[75] * state[76] - state[73] * state[74]));
  xx[4] = (xx[17] == 0.0 && xx[4] == 0.0) ? 0.0 : atan2(xx[17], xx[4]);
  xx[12] = 0.5;
  xx[17] = - (xx[2] * (xx[0] - xx[1]) * xx[15]);
  xx[18] = (xx[3] + xx[11]) * xx[2] * xx[15];
  xx[17] = (xx[18] == 0.0 && xx[17] == 0.0) ? 0.0 : atan2(xx[18], xx[17]);
  xx[0] = bb[0] ? xx[4] : xx[12] * xx[17];
  xx[1] = bb[0] ? xx[14] : xx[15] * xx[0];
  xx[3] = fabs(xx[7]) - xx[10];
  if (xx[3] < 0.0)
    ii[0] = -1;
  else if (xx[3] > 0.0)
    ii[0] = +1;
  else
    ii[0] = 0;
  ii[1] = ii[0];
  if (0 > ii[1])
    ii[1] = 0;
  bb[0] = !(ii[1] == 1);
  xx[3] = origState[74] * origState[75];
  xx[4] = origState[73] * origState[76];
  xx[7] = origState[73] * origState[73];
  xx[10] = (xx[7] + origState[74] * origState[74]) * xx[2] - xx[13];
  xx[11] = - (xx[2] * (xx[3] - xx[4]));
  xx[10] = (xx[11] == 0.0 && xx[10] == 0.0) ? 0.0 : atan2(xx[11], xx[10]);
  if (xx[8] < 0.0)
    xx[11] = -1.0;
  else if (xx[8] > 0.0)
    xx[11] = +1.0;
  else
    xx[11] = 0.0;
  xx[8] = (xx[7] + origState[76] * origState[76]) * xx[2] - xx[13];
  xx[14] = - (xx[2] * (origState[75] * origState[76] - origState[73] *
                       origState[74]));
  xx[8] = (xx[14] == 0.0 && xx[8] == 0.0) ? 0.0 : atan2(xx[14], xx[8]);
  xx[7] = - (xx[2] * (xx[5] - xx[6]) * xx[11]);
  xx[13] = (xx[3] + xx[4]) * xx[2] * xx[11];
  xx[7] = (xx[13] == 0.0 && xx[7] == 0.0) ? 0.0 : atan2(xx[13], xx[7]);
  xx[2] = bb[0] ? xx[8] : xx[12] * xx[7];
  xx[3] = bb[0] ? xx[10] : xx[11] * xx[2];
  xx[17] = xx[1];
  xx[18] = xx[1];
  xx[19] = xx[1];
  xx[20] = xx[1];
  xx[21] = xx[3];
  xx[22] = xx[3];
  xx[23] = xx[3];
  xx[1] = xx[17 + (partialType)];
  xx[3] = cos(xx[1]);
  xx[4] = sin(xx[1]);
  xx[1] = sin(xx[16]);
  xx[10] = xx[0];
  xx[11] = xx[2];
  xx[12] = xx[0];
  xx[13] = xx[2];
  xx[14] = xx[0];
  xx[15] = xx[2];
  xx[16] = xx[0];
  xx[0] = xx[10 + (partialType)];
  xx[2] = cos(xx[0]);
  xx[5] = sin(xx[0]);
  xx[0] = xx[3] * xx[5];
  xx[6] = xx[3] * xx[2];
  xx[10] = xx[9] * xx[3];
  xx[11] = - (xx[9] * xx[4]);
  xx[12] = xx[1];
  xx[13] = xx[2] * xx[4] + xx[0] * xx[1];
  xx[14] = xx[6] - xx[1] * xx[5] * xx[4];
  xx[15] = - (xx[9] * xx[5]);
  xx[16] = xx[4] * xx[5] - xx[6] * xx[1];
  xx[17] = xx[0] + xx[2] * xx[1] * xx[4];
  xx[18] = xx[9] * xx[2];
  pm_math_Quaternion_Matrix3x3Ctor_ra(xx + 10, xx + 0);
  state[73] = xx[0];
  state[74] = xx[1];
  state[75] = xx[2];
  state[76] = xx[3];
}

static void projectPartiallyTargetedPos_12_0(const double *origState, int
  partialType, double *state)
{
  boolean_T bb[1];
  int ii[3];
  double xx[24];
  xx[0] = state[81] * state[83];
  xx[1] = state[80] * state[82];
  xx[2] = 2.0;
  xx[3] = (xx[0] + xx[1]) * xx[2];
  xx[4] = fabs(xx[3]) > 1.0 ? atan2(xx[3], 0.0) : asin(xx[3]);
  xx[5] = origState[81] * origState[83];
  xx[6] = origState[80] * origState[82];
  xx[7] = (xx[5] + xx[6]) * xx[2];
  xx[8] = fabs(xx[7]) > 1.0 ? atan2(xx[7], 0.0) : asin(xx[7]);
  xx[9] = xx[4];
  xx[10] = xx[4];
  xx[11] = xx[8];
  xx[12] = xx[8];
  xx[13] = xx[4];
  xx[14] = xx[4];
  xx[15] = xx[8];
  xx[16] = xx[9 + (partialType)];
  xx[9] = cos(xx[16]);
  xx[10] = 0.99999999999999;
  xx[11] = fabs(xx[3]) - xx[10];
  if (xx[11] < 0.0)
    ii[0] = -1;
  else if (xx[11] > 0.0)
    ii[0] = +1;
  else
    ii[0] = 0;
  ii[1] = ii[0];
  if (0 > ii[1])
    ii[1] = 0;
  bb[0] = !(ii[1] == 1);
  xx[3] = state[81] * state[82];
  xx[11] = state[80] * state[83];
  xx[12] = state[80] * state[80];
  xx[13] = 1.0;
  xx[14] = (xx[12] + state[81] * state[81]) * xx[2] - xx[13];
  xx[15] = - (xx[2] * (xx[3] - xx[11]));
  xx[14] = (xx[15] == 0.0 && xx[14] == 0.0) ? 0.0 : atan2(xx[15], xx[14]);
  if (xx[4] < 0.0)
    xx[15] = -1.0;
  else if (xx[4] > 0.0)
    xx[15] = +1.0;
  else
    xx[15] = 0.0;
  xx[4] = (xx[12] + state[83] * state[83]) * xx[2] - xx[13];
  xx[17] = - (xx[2] * (state[82] * state[83] - state[80] * state[81]));
  xx[4] = (xx[17] == 0.0 && xx[4] == 0.0) ? 0.0 : atan2(xx[17], xx[4]);
  xx[12] = 0.5;
  xx[17] = - (xx[2] * (xx[0] - xx[1]) * xx[15]);
  xx[18] = (xx[3] + xx[11]) * xx[2] * xx[15];
  xx[17] = (xx[18] == 0.0 && xx[17] == 0.0) ? 0.0 : atan2(xx[18], xx[17]);
  xx[0] = bb[0] ? xx[4] : xx[12] * xx[17];
  xx[1] = bb[0] ? xx[14] : xx[15] * xx[0];
  xx[3] = fabs(xx[7]) - xx[10];
  if (xx[3] < 0.0)
    ii[0] = -1;
  else if (xx[3] > 0.0)
    ii[0] = +1;
  else
    ii[0] = 0;
  ii[1] = ii[0];
  if (0 > ii[1])
    ii[1] = 0;
  bb[0] = !(ii[1] == 1);
  xx[3] = origState[81] * origState[82];
  xx[4] = origState[80] * origState[83];
  xx[7] = origState[80] * origState[80];
  xx[10] = (xx[7] + origState[81] * origState[81]) * xx[2] - xx[13];
  xx[11] = - (xx[2] * (xx[3] - xx[4]));
  xx[10] = (xx[11] == 0.0 && xx[10] == 0.0) ? 0.0 : atan2(xx[11], xx[10]);
  if (xx[8] < 0.0)
    xx[11] = -1.0;
  else if (xx[8] > 0.0)
    xx[11] = +1.0;
  else
    xx[11] = 0.0;
  xx[8] = (xx[7] + origState[83] * origState[83]) * xx[2] - xx[13];
  xx[14] = - (xx[2] * (origState[82] * origState[83] - origState[80] *
                       origState[81]));
  xx[8] = (xx[14] == 0.0 && xx[8] == 0.0) ? 0.0 : atan2(xx[14], xx[8]);
  xx[7] = - (xx[2] * (xx[5] - xx[6]) * xx[11]);
  xx[13] = (xx[3] + xx[4]) * xx[2] * xx[11];
  xx[7] = (xx[13] == 0.0 && xx[7] == 0.0) ? 0.0 : atan2(xx[13], xx[7]);
  xx[2] = bb[0] ? xx[8] : xx[12] * xx[7];
  xx[3] = bb[0] ? xx[10] : xx[11] * xx[2];
  xx[17] = xx[1];
  xx[18] = xx[1];
  xx[19] = xx[1];
  xx[20] = xx[1];
  xx[21] = xx[3];
  xx[22] = xx[3];
  xx[23] = xx[3];
  xx[1] = xx[17 + (partialType)];
  xx[3] = cos(xx[1]);
  xx[4] = sin(xx[1]);
  xx[1] = sin(xx[16]);
  xx[10] = xx[0];
  xx[11] = xx[2];
  xx[12] = xx[0];
  xx[13] = xx[2];
  xx[14] = xx[0];
  xx[15] = xx[2];
  xx[16] = xx[0];
  xx[0] = xx[10 + (partialType)];
  xx[2] = cos(xx[0]);
  xx[5] = sin(xx[0]);
  xx[0] = xx[3] * xx[5];
  xx[6] = xx[3] * xx[2];
  xx[10] = xx[9] * xx[3];
  xx[11] = - (xx[9] * xx[4]);
  xx[12] = xx[1];
  xx[13] = xx[2] * xx[4] + xx[0] * xx[1];
  xx[14] = xx[6] - xx[1] * xx[5] * xx[4];
  xx[15] = - (xx[9] * xx[5]);
  xx[16] = xx[4] * xx[5] - xx[6] * xx[1];
  xx[17] = xx[0] + xx[2] * xx[1] * xx[4];
  xx[18] = xx[9] * xx[2];
  pm_math_Quaternion_Matrix3x3Ctor_ra(xx + 10, xx + 0);
  state[80] = xx[0];
  state[81] = xx[1];
  state[82] = xx[2];
  state[83] = xx[3];
}

static void projectPartiallyTargetedPos_13_0(const double *origState, int
  partialType, double *state)
{
  boolean_T bb[1];
  int ii[3];
  double xx[24];
  xx[0] = state[88] * state[90];
  xx[1] = state[87] * state[89];
  xx[2] = 2.0;
  xx[3] = (xx[0] + xx[1]) * xx[2];
  xx[4] = fabs(xx[3]) > 1.0 ? atan2(xx[3], 0.0) : asin(xx[3]);
  xx[5] = origState[88] * origState[90];
  xx[6] = origState[87] * origState[89];
  xx[7] = (xx[5] + xx[6]) * xx[2];
  xx[8] = fabs(xx[7]) > 1.0 ? atan2(xx[7], 0.0) : asin(xx[7]);
  xx[9] = xx[4];
  xx[10] = xx[4];
  xx[11] = xx[8];
  xx[12] = xx[8];
  xx[13] = xx[4];
  xx[14] = xx[4];
  xx[15] = xx[8];
  xx[16] = xx[9 + (partialType)];
  xx[9] = cos(xx[16]);
  xx[10] = 0.99999999999999;
  xx[11] = fabs(xx[3]) - xx[10];
  if (xx[11] < 0.0)
    ii[0] = -1;
  else if (xx[11] > 0.0)
    ii[0] = +1;
  else
    ii[0] = 0;
  ii[1] = ii[0];
  if (0 > ii[1])
    ii[1] = 0;
  bb[0] = !(ii[1] == 1);
  xx[3] = state[88] * state[89];
  xx[11] = state[87] * state[90];
  xx[12] = state[87] * state[87];
  xx[13] = 1.0;
  xx[14] = (xx[12] + state[88] * state[88]) * xx[2] - xx[13];
  xx[15] = - (xx[2] * (xx[3] - xx[11]));
  xx[14] = (xx[15] == 0.0 && xx[14] == 0.0) ? 0.0 : atan2(xx[15], xx[14]);
  if (xx[4] < 0.0)
    xx[15] = -1.0;
  else if (xx[4] > 0.0)
    xx[15] = +1.0;
  else
    xx[15] = 0.0;
  xx[4] = (xx[12] + state[90] * state[90]) * xx[2] - xx[13];
  xx[17] = - (xx[2] * (state[89] * state[90] - state[87] * state[88]));
  xx[4] = (xx[17] == 0.0 && xx[4] == 0.0) ? 0.0 : atan2(xx[17], xx[4]);
  xx[12] = 0.5;
  xx[17] = - (xx[2] * (xx[0] - xx[1]) * xx[15]);
  xx[18] = (xx[3] + xx[11]) * xx[2] * xx[15];
  xx[17] = (xx[18] == 0.0 && xx[17] == 0.0) ? 0.0 : atan2(xx[18], xx[17]);
  xx[0] = bb[0] ? xx[4] : xx[12] * xx[17];
  xx[1] = bb[0] ? xx[14] : xx[15] * xx[0];
  xx[3] = fabs(xx[7]) - xx[10];
  if (xx[3] < 0.0)
    ii[0] = -1;
  else if (xx[3] > 0.0)
    ii[0] = +1;
  else
    ii[0] = 0;
  ii[1] = ii[0];
  if (0 > ii[1])
    ii[1] = 0;
  bb[0] = !(ii[1] == 1);
  xx[3] = origState[88] * origState[89];
  xx[4] = origState[87] * origState[90];
  xx[7] = origState[87] * origState[87];
  xx[10] = (xx[7] + origState[88] * origState[88]) * xx[2] - xx[13];
  xx[11] = - (xx[2] * (xx[3] - xx[4]));
  xx[10] = (xx[11] == 0.0 && xx[10] == 0.0) ? 0.0 : atan2(xx[11], xx[10]);
  if (xx[8] < 0.0)
    xx[11] = -1.0;
  else if (xx[8] > 0.0)
    xx[11] = +1.0;
  else
    xx[11] = 0.0;
  xx[8] = (xx[7] + origState[90] * origState[90]) * xx[2] - xx[13];
  xx[14] = - (xx[2] * (origState[89] * origState[90] - origState[87] *
                       origState[88]));
  xx[8] = (xx[14] == 0.0 && xx[8] == 0.0) ? 0.0 : atan2(xx[14], xx[8]);
  xx[7] = - (xx[2] * (xx[5] - xx[6]) * xx[11]);
  xx[13] = (xx[3] + xx[4]) * xx[2] * xx[11];
  xx[7] = (xx[13] == 0.0 && xx[7] == 0.0) ? 0.0 : atan2(xx[13], xx[7]);
  xx[2] = bb[0] ? xx[8] : xx[12] * xx[7];
  xx[3] = bb[0] ? xx[10] : xx[11] * xx[2];
  xx[17] = xx[1];
  xx[18] = xx[1];
  xx[19] = xx[1];
  xx[20] = xx[1];
  xx[21] = xx[3];
  xx[22] = xx[3];
  xx[23] = xx[3];
  xx[1] = xx[17 + (partialType)];
  xx[3] = cos(xx[1]);
  xx[4] = sin(xx[1]);
  xx[1] = sin(xx[16]);
  xx[10] = xx[0];
  xx[11] = xx[2];
  xx[12] = xx[0];
  xx[13] = xx[2];
  xx[14] = xx[0];
  xx[15] = xx[2];
  xx[16] = xx[0];
  xx[0] = xx[10 + (partialType)];
  xx[2] = cos(xx[0]);
  xx[5] = sin(xx[0]);
  xx[0] = xx[3] * xx[5];
  xx[6] = xx[3] * xx[2];
  xx[10] = xx[9] * xx[3];
  xx[11] = - (xx[9] * xx[4]);
  xx[12] = xx[1];
  xx[13] = xx[2] * xx[4] + xx[0] * xx[1];
  xx[14] = xx[6] - xx[1] * xx[5] * xx[4];
  xx[15] = - (xx[9] * xx[5]);
  xx[16] = xx[4] * xx[5] - xx[6] * xx[1];
  xx[17] = xx[0] + xx[2] * xx[1] * xx[4];
  xx[18] = xx[9] * xx[2];
  pm_math_Quaternion_Matrix3x3Ctor_ra(xx + 10, xx + 0);
  state[87] = xx[0];
  state[88] = xx[1];
  state[89] = xx[2];
  state[90] = xx[3];
}

static void projectPartiallyTargetedPos_16_0(const double *origState, int
  partialType, double *state)
{
  boolean_T bb[1];
  int ii[3];
  double xx[24];
  xx[0] = state[99] * state[101];
  xx[1] = state[98] * state[100];
  xx[2] = 2.0;
  xx[3] = (xx[0] + xx[1]) * xx[2];
  xx[4] = fabs(xx[3]) > 1.0 ? atan2(xx[3], 0.0) : asin(xx[3]);
  xx[5] = origState[99] * origState[101];
  xx[6] = origState[98] * origState[100];
  xx[7] = (xx[5] + xx[6]) * xx[2];
  xx[8] = fabs(xx[7]) > 1.0 ? atan2(xx[7], 0.0) : asin(xx[7]);
  xx[9] = xx[4];
  xx[10] = xx[4];
  xx[11] = xx[8];
  xx[12] = xx[8];
  xx[13] = xx[4];
  xx[14] = xx[4];
  xx[15] = xx[8];
  xx[16] = xx[9 + (partialType)];
  xx[9] = cos(xx[16]);
  xx[10] = 0.99999999999999;
  xx[11] = fabs(xx[3]) - xx[10];
  if (xx[11] < 0.0)
    ii[0] = -1;
  else if (xx[11] > 0.0)
    ii[0] = +1;
  else
    ii[0] = 0;
  ii[1] = ii[0];
  if (0 > ii[1])
    ii[1] = 0;
  bb[0] = !(ii[1] == 1);
  xx[3] = state[99] * state[100];
  xx[11] = state[98] * state[101];
  xx[12] = state[98] * state[98];
  xx[13] = 1.0;
  xx[14] = (xx[12] + state[99] * state[99]) * xx[2] - xx[13];
  xx[15] = - (xx[2] * (xx[3] - xx[11]));
  xx[14] = (xx[15] == 0.0 && xx[14] == 0.0) ? 0.0 : atan2(xx[15], xx[14]);
  if (xx[4] < 0.0)
    xx[15] = -1.0;
  else if (xx[4] > 0.0)
    xx[15] = +1.0;
  else
    xx[15] = 0.0;
  xx[4] = (xx[12] + state[101] * state[101]) * xx[2] - xx[13];
  xx[17] = - (xx[2] * (state[100] * state[101] - state[98] * state[99]));
  xx[4] = (xx[17] == 0.0 && xx[4] == 0.0) ? 0.0 : atan2(xx[17], xx[4]);
  xx[12] = 0.5;
  xx[17] = - (xx[2] * (xx[0] - xx[1]) * xx[15]);
  xx[18] = (xx[3] + xx[11]) * xx[2] * xx[15];
  xx[17] = (xx[18] == 0.0 && xx[17] == 0.0) ? 0.0 : atan2(xx[18], xx[17]);
  xx[0] = bb[0] ? xx[4] : xx[12] * xx[17];
  xx[1] = bb[0] ? xx[14] : xx[15] * xx[0];
  xx[3] = fabs(xx[7]) - xx[10];
  if (xx[3] < 0.0)
    ii[0] = -1;
  else if (xx[3] > 0.0)
    ii[0] = +1;
  else
    ii[0] = 0;
  ii[1] = ii[0];
  if (0 > ii[1])
    ii[1] = 0;
  bb[0] = !(ii[1] == 1);
  xx[3] = origState[99] * origState[100];
  xx[4] = origState[98] * origState[101];
  xx[7] = origState[98] * origState[98];
  xx[10] = (xx[7] + origState[99] * origState[99]) * xx[2] - xx[13];
  xx[11] = - (xx[2] * (xx[3] - xx[4]));
  xx[10] = (xx[11] == 0.0 && xx[10] == 0.0) ? 0.0 : atan2(xx[11], xx[10]);
  if (xx[8] < 0.0)
    xx[11] = -1.0;
  else if (xx[8] > 0.0)
    xx[11] = +1.0;
  else
    xx[11] = 0.0;
  xx[8] = (xx[7] + origState[101] * origState[101]) * xx[2] - xx[13];
  xx[14] = - (xx[2] * (origState[100] * origState[101] - origState[98] *
                       origState[99]));
  xx[8] = (xx[14] == 0.0 && xx[8] == 0.0) ? 0.0 : atan2(xx[14], xx[8]);
  xx[7] = - (xx[2] * (xx[5] - xx[6]) * xx[11]);
  xx[13] = (xx[3] + xx[4]) * xx[2] * xx[11];
  xx[7] = (xx[13] == 0.0 && xx[7] == 0.0) ? 0.0 : atan2(xx[13], xx[7]);
  xx[2] = bb[0] ? xx[8] : xx[12] * xx[7];
  xx[3] = bb[0] ? xx[10] : xx[11] * xx[2];
  xx[17] = xx[1];
  xx[18] = xx[1];
  xx[19] = xx[1];
  xx[20] = xx[1];
  xx[21] = xx[3];
  xx[22] = xx[3];
  xx[23] = xx[3];
  xx[1] = xx[17 + (partialType)];
  xx[3] = cos(xx[1]);
  xx[4] = sin(xx[1]);
  xx[1] = sin(xx[16]);
  xx[10] = xx[0];
  xx[11] = xx[2];
  xx[12] = xx[0];
  xx[13] = xx[2];
  xx[14] = xx[0];
  xx[15] = xx[2];
  xx[16] = xx[0];
  xx[0] = xx[10 + (partialType)];
  xx[2] = cos(xx[0]);
  xx[5] = sin(xx[0]);
  xx[0] = xx[3] * xx[5];
  xx[6] = xx[3] * xx[2];
  xx[10] = xx[9] * xx[3];
  xx[11] = - (xx[9] * xx[4]);
  xx[12] = xx[1];
  xx[13] = xx[2] * xx[4] + xx[0] * xx[1];
  xx[14] = xx[6] - xx[1] * xx[5] * xx[4];
  xx[15] = - (xx[9] * xx[5]);
  xx[16] = xx[4] * xx[5] - xx[6] * xx[1];
  xx[17] = xx[0] + xx[2] * xx[1] * xx[4];
  xx[18] = xx[9] * xx[2];
  pm_math_Quaternion_Matrix3x3Ctor_ra(xx + 10, xx + 0);
  state[98] = xx[0];
  state[99] = xx[1];
  state[100] = xx[2];
  state[101] = xx[3];
}

static void projectPartiallyTargetedPos_17_0(const double *origState, int
  partialType, double *state)
{
  boolean_T bb[1];
  int ii[3];
  double xx[24];
  xx[0] = state[106] * state[108];
  xx[1] = state[105] * state[107];
  xx[2] = 2.0;
  xx[3] = (xx[0] + xx[1]) * xx[2];
  xx[4] = fabs(xx[3]) > 1.0 ? atan2(xx[3], 0.0) : asin(xx[3]);
  xx[5] = origState[106] * origState[108];
  xx[6] = origState[105] * origState[107];
  xx[7] = (xx[5] + xx[6]) * xx[2];
  xx[8] = fabs(xx[7]) > 1.0 ? atan2(xx[7], 0.0) : asin(xx[7]);
  xx[9] = xx[4];
  xx[10] = xx[4];
  xx[11] = xx[8];
  xx[12] = xx[8];
  xx[13] = xx[4];
  xx[14] = xx[4];
  xx[15] = xx[8];
  xx[16] = xx[9 + (partialType)];
  xx[9] = cos(xx[16]);
  xx[10] = 0.99999999999999;
  xx[11] = fabs(xx[3]) - xx[10];
  if (xx[11] < 0.0)
    ii[0] = -1;
  else if (xx[11] > 0.0)
    ii[0] = +1;
  else
    ii[0] = 0;
  ii[1] = ii[0];
  if (0 > ii[1])
    ii[1] = 0;
  bb[0] = !(ii[1] == 1);
  xx[3] = state[106] * state[107];
  xx[11] = state[105] * state[108];
  xx[12] = state[105] * state[105];
  xx[13] = 1.0;
  xx[14] = (xx[12] + state[106] * state[106]) * xx[2] - xx[13];
  xx[15] = - (xx[2] * (xx[3] - xx[11]));
  xx[14] = (xx[15] == 0.0 && xx[14] == 0.0) ? 0.0 : atan2(xx[15], xx[14]);
  if (xx[4] < 0.0)
    xx[15] = -1.0;
  else if (xx[4] > 0.0)
    xx[15] = +1.0;
  else
    xx[15] = 0.0;
  xx[4] = (xx[12] + state[108] * state[108]) * xx[2] - xx[13];
  xx[17] = - (xx[2] * (state[107] * state[108] - state[105] * state[106]));
  xx[4] = (xx[17] == 0.0 && xx[4] == 0.0) ? 0.0 : atan2(xx[17], xx[4]);
  xx[12] = 0.5;
  xx[17] = - (xx[2] * (xx[0] - xx[1]) * xx[15]);
  xx[18] = (xx[3] + xx[11]) * xx[2] * xx[15];
  xx[17] = (xx[18] == 0.0 && xx[17] == 0.0) ? 0.0 : atan2(xx[18], xx[17]);
  xx[0] = bb[0] ? xx[4] : xx[12] * xx[17];
  xx[1] = bb[0] ? xx[14] : xx[15] * xx[0];
  xx[3] = fabs(xx[7]) - xx[10];
  if (xx[3] < 0.0)
    ii[0] = -1;
  else if (xx[3] > 0.0)
    ii[0] = +1;
  else
    ii[0] = 0;
  ii[1] = ii[0];
  if (0 > ii[1])
    ii[1] = 0;
  bb[0] = !(ii[1] == 1);
  xx[3] = origState[106] * origState[107];
  xx[4] = origState[105] * origState[108];
  xx[7] = origState[105] * origState[105];
  xx[10] = (xx[7] + origState[106] * origState[106]) * xx[2] - xx[13];
  xx[11] = - (xx[2] * (xx[3] - xx[4]));
  xx[10] = (xx[11] == 0.0 && xx[10] == 0.0) ? 0.0 : atan2(xx[11], xx[10]);
  if (xx[8] < 0.0)
    xx[11] = -1.0;
  else if (xx[8] > 0.0)
    xx[11] = +1.0;
  else
    xx[11] = 0.0;
  xx[8] = (xx[7] + origState[108] * origState[108]) * xx[2] - xx[13];
  xx[14] = - (xx[2] * (origState[107] * origState[108] - origState[105] *
                       origState[106]));
  xx[8] = (xx[14] == 0.0 && xx[8] == 0.0) ? 0.0 : atan2(xx[14], xx[8]);
  xx[7] = - (xx[2] * (xx[5] - xx[6]) * xx[11]);
  xx[13] = (xx[3] + xx[4]) * xx[2] * xx[11];
  xx[7] = (xx[13] == 0.0 && xx[7] == 0.0) ? 0.0 : atan2(xx[13], xx[7]);
  xx[2] = bb[0] ? xx[8] : xx[12] * xx[7];
  xx[3] = bb[0] ? xx[10] : xx[11] * xx[2];
  xx[17] = xx[1];
  xx[18] = xx[1];
  xx[19] = xx[1];
  xx[20] = xx[1];
  xx[21] = xx[3];
  xx[22] = xx[3];
  xx[23] = xx[3];
  xx[1] = xx[17 + (partialType)];
  xx[3] = cos(xx[1]);
  xx[4] = sin(xx[1]);
  xx[1] = sin(xx[16]);
  xx[10] = xx[0];
  xx[11] = xx[2];
  xx[12] = xx[0];
  xx[13] = xx[2];
  xx[14] = xx[0];
  xx[15] = xx[2];
  xx[16] = xx[0];
  xx[0] = xx[10 + (partialType)];
  xx[2] = cos(xx[0]);
  xx[5] = sin(xx[0]);
  xx[0] = xx[3] * xx[5];
  xx[6] = xx[3] * xx[2];
  xx[10] = xx[9] * xx[3];
  xx[11] = - (xx[9] * xx[4]);
  xx[12] = xx[1];
  xx[13] = xx[2] * xx[4] + xx[0] * xx[1];
  xx[14] = xx[6] - xx[1] * xx[5] * xx[4];
  xx[15] = - (xx[9] * xx[5]);
  xx[16] = xx[4] * xx[5] - xx[6] * xx[1];
  xx[17] = xx[0] + xx[2] * xx[1] * xx[4];
  xx[18] = xx[9] * xx[2];
  pm_math_Quaternion_Matrix3x3Ctor_ra(xx + 10, xx + 0);
  state[105] = xx[0];
  state[106] = xx[1];
  state[107] = xx[2];
  state[108] = xx[3];
}

static void projectPartiallyTargetedPos_18_0(const double *origState, int
  partialType, double *state)
{
  boolean_T bb[1];
  int ii[3];
  double xx[24];
  xx[0] = state[113] * state[115];
  xx[1] = state[112] * state[114];
  xx[2] = 2.0;
  xx[3] = (xx[0] + xx[1]) * xx[2];
  xx[4] = fabs(xx[3]) > 1.0 ? atan2(xx[3], 0.0) : asin(xx[3]);
  xx[5] = origState[113] * origState[115];
  xx[6] = origState[112] * origState[114];
  xx[7] = (xx[5] + xx[6]) * xx[2];
  xx[8] = fabs(xx[7]) > 1.0 ? atan2(xx[7], 0.0) : asin(xx[7]);
  xx[9] = xx[4];
  xx[10] = xx[4];
  xx[11] = xx[8];
  xx[12] = xx[8];
  xx[13] = xx[4];
  xx[14] = xx[4];
  xx[15] = xx[8];
  xx[16] = xx[9 + (partialType)];
  xx[9] = cos(xx[16]);
  xx[10] = 0.99999999999999;
  xx[11] = fabs(xx[3]) - xx[10];
  if (xx[11] < 0.0)
    ii[0] = -1;
  else if (xx[11] > 0.0)
    ii[0] = +1;
  else
    ii[0] = 0;
  ii[1] = ii[0];
  if (0 > ii[1])
    ii[1] = 0;
  bb[0] = !(ii[1] == 1);
  xx[3] = state[113] * state[114];
  xx[11] = state[112] * state[115];
  xx[12] = state[112] * state[112];
  xx[13] = 1.0;
  xx[14] = (xx[12] + state[113] * state[113]) * xx[2] - xx[13];
  xx[15] = - (xx[2] * (xx[3] - xx[11]));
  xx[14] = (xx[15] == 0.0 && xx[14] == 0.0) ? 0.0 : atan2(xx[15], xx[14]);
  if (xx[4] < 0.0)
    xx[15] = -1.0;
  else if (xx[4] > 0.0)
    xx[15] = +1.0;
  else
    xx[15] = 0.0;
  xx[4] = (xx[12] + state[115] * state[115]) * xx[2] - xx[13];
  xx[17] = - (xx[2] * (state[114] * state[115] - state[112] * state[113]));
  xx[4] = (xx[17] == 0.0 && xx[4] == 0.0) ? 0.0 : atan2(xx[17], xx[4]);
  xx[12] = 0.5;
  xx[17] = - (xx[2] * (xx[0] - xx[1]) * xx[15]);
  xx[18] = (xx[3] + xx[11]) * xx[2] * xx[15];
  xx[17] = (xx[18] == 0.0 && xx[17] == 0.0) ? 0.0 : atan2(xx[18], xx[17]);
  xx[0] = bb[0] ? xx[4] : xx[12] * xx[17];
  xx[1] = bb[0] ? xx[14] : xx[15] * xx[0];
  xx[3] = fabs(xx[7]) - xx[10];
  if (xx[3] < 0.0)
    ii[0] = -1;
  else if (xx[3] > 0.0)
    ii[0] = +1;
  else
    ii[0] = 0;
  ii[1] = ii[0];
  if (0 > ii[1])
    ii[1] = 0;
  bb[0] = !(ii[1] == 1);
  xx[3] = origState[113] * origState[114];
  xx[4] = origState[112] * origState[115];
  xx[7] = origState[112] * origState[112];
  xx[10] = (xx[7] + origState[113] * origState[113]) * xx[2] - xx[13];
  xx[11] = - (xx[2] * (xx[3] - xx[4]));
  xx[10] = (xx[11] == 0.0 && xx[10] == 0.0) ? 0.0 : atan2(xx[11], xx[10]);
  if (xx[8] < 0.0)
    xx[11] = -1.0;
  else if (xx[8] > 0.0)
    xx[11] = +1.0;
  else
    xx[11] = 0.0;
  xx[8] = (xx[7] + origState[115] * origState[115]) * xx[2] - xx[13];
  xx[14] = - (xx[2] * (origState[114] * origState[115] - origState[112] *
                       origState[113]));
  xx[8] = (xx[14] == 0.0 && xx[8] == 0.0) ? 0.0 : atan2(xx[14], xx[8]);
  xx[7] = - (xx[2] * (xx[5] - xx[6]) * xx[11]);
  xx[13] = (xx[3] + xx[4]) * xx[2] * xx[11];
  xx[7] = (xx[13] == 0.0 && xx[7] == 0.0) ? 0.0 : atan2(xx[13], xx[7]);
  xx[2] = bb[0] ? xx[8] : xx[12] * xx[7];
  xx[3] = bb[0] ? xx[10] : xx[11] * xx[2];
  xx[17] = xx[1];
  xx[18] = xx[1];
  xx[19] = xx[1];
  xx[20] = xx[1];
  xx[21] = xx[3];
  xx[22] = xx[3];
  xx[23] = xx[3];
  xx[1] = xx[17 + (partialType)];
  xx[3] = cos(xx[1]);
  xx[4] = sin(xx[1]);
  xx[1] = sin(xx[16]);
  xx[10] = xx[0];
  xx[11] = xx[2];
  xx[12] = xx[0];
  xx[13] = xx[2];
  xx[14] = xx[0];
  xx[15] = xx[2];
  xx[16] = xx[0];
  xx[0] = xx[10 + (partialType)];
  xx[2] = cos(xx[0]);
  xx[5] = sin(xx[0]);
  xx[0] = xx[3] * xx[5];
  xx[6] = xx[3] * xx[2];
  xx[10] = xx[9] * xx[3];
  xx[11] = - (xx[9] * xx[4]);
  xx[12] = xx[1];
  xx[13] = xx[2] * xx[4] + xx[0] * xx[1];
  xx[14] = xx[6] - xx[1] * xx[5] * xx[4];
  xx[15] = - (xx[9] * xx[5]);
  xx[16] = xx[4] * xx[5] - xx[6] * xx[1];
  xx[17] = xx[0] + xx[2] * xx[1] * xx[4];
  xx[18] = xx[9] * xx[2];
  pm_math_Quaternion_Matrix3x3Ctor_ra(xx + 10, xx + 0);
  state[112] = xx[0];
  state[113] = xx[1];
  state[114] = xx[2];
  state[115] = xx[3];
}

static void projectPartiallyTargetedPos_19_0(const double *origState, int
  partialType, double *state)
{
  boolean_T bb[1];
  int ii[3];
  double xx[24];
  xx[0] = state[120] * state[122];
  xx[1] = state[119] * state[121];
  xx[2] = 2.0;
  xx[3] = (xx[0] + xx[1]) * xx[2];
  xx[4] = fabs(xx[3]) > 1.0 ? atan2(xx[3], 0.0) : asin(xx[3]);
  xx[5] = origState[120] * origState[122];
  xx[6] = origState[119] * origState[121];
  xx[7] = (xx[5] + xx[6]) * xx[2];
  xx[8] = fabs(xx[7]) > 1.0 ? atan2(xx[7], 0.0) : asin(xx[7]);
  xx[9] = xx[4];
  xx[10] = xx[4];
  xx[11] = xx[8];
  xx[12] = xx[8];
  xx[13] = xx[4];
  xx[14] = xx[4];
  xx[15] = xx[8];
  xx[16] = xx[9 + (partialType)];
  xx[9] = cos(xx[16]);
  xx[10] = 0.99999999999999;
  xx[11] = fabs(xx[3]) - xx[10];
  if (xx[11] < 0.0)
    ii[0] = -1;
  else if (xx[11] > 0.0)
    ii[0] = +1;
  else
    ii[0] = 0;
  ii[1] = ii[0];
  if (0 > ii[1])
    ii[1] = 0;
  bb[0] = !(ii[1] == 1);
  xx[3] = state[120] * state[121];
  xx[11] = state[119] * state[122];
  xx[12] = state[119] * state[119];
  xx[13] = 1.0;
  xx[14] = (xx[12] + state[120] * state[120]) * xx[2] - xx[13];
  xx[15] = - (xx[2] * (xx[3] - xx[11]));
  xx[14] = (xx[15] == 0.0 && xx[14] == 0.0) ? 0.0 : atan2(xx[15], xx[14]);
  if (xx[4] < 0.0)
    xx[15] = -1.0;
  else if (xx[4] > 0.0)
    xx[15] = +1.0;
  else
    xx[15] = 0.0;
  xx[4] = (xx[12] + state[122] * state[122]) * xx[2] - xx[13];
  xx[17] = - (xx[2] * (state[121] * state[122] - state[119] * state[120]));
  xx[4] = (xx[17] == 0.0 && xx[4] == 0.0) ? 0.0 : atan2(xx[17], xx[4]);
  xx[12] = 0.5;
  xx[17] = - (xx[2] * (xx[0] - xx[1]) * xx[15]);
  xx[18] = (xx[3] + xx[11]) * xx[2] * xx[15];
  xx[17] = (xx[18] == 0.0 && xx[17] == 0.0) ? 0.0 : atan2(xx[18], xx[17]);
  xx[0] = bb[0] ? xx[4] : xx[12] * xx[17];
  xx[1] = bb[0] ? xx[14] : xx[15] * xx[0];
  xx[3] = fabs(xx[7]) - xx[10];
  if (xx[3] < 0.0)
    ii[0] = -1;
  else if (xx[3] > 0.0)
    ii[0] = +1;
  else
    ii[0] = 0;
  ii[1] = ii[0];
  if (0 > ii[1])
    ii[1] = 0;
  bb[0] = !(ii[1] == 1);
  xx[3] = origState[120] * origState[121];
  xx[4] = origState[119] * origState[122];
  xx[7] = origState[119] * origState[119];
  xx[10] = (xx[7] + origState[120] * origState[120]) * xx[2] - xx[13];
  xx[11] = - (xx[2] * (xx[3] - xx[4]));
  xx[10] = (xx[11] == 0.0 && xx[10] == 0.0) ? 0.0 : atan2(xx[11], xx[10]);
  if (xx[8] < 0.0)
    xx[11] = -1.0;
  else if (xx[8] > 0.0)
    xx[11] = +1.0;
  else
    xx[11] = 0.0;
  xx[8] = (xx[7] + origState[122] * origState[122]) * xx[2] - xx[13];
  xx[14] = - (xx[2] * (origState[121] * origState[122] - origState[119] *
                       origState[120]));
  xx[8] = (xx[14] == 0.0 && xx[8] == 0.0) ? 0.0 : atan2(xx[14], xx[8]);
  xx[7] = - (xx[2] * (xx[5] - xx[6]) * xx[11]);
  xx[13] = (xx[3] + xx[4]) * xx[2] * xx[11];
  xx[7] = (xx[13] == 0.0 && xx[7] == 0.0) ? 0.0 : atan2(xx[13], xx[7]);
  xx[2] = bb[0] ? xx[8] : xx[12] * xx[7];
  xx[3] = bb[0] ? xx[10] : xx[11] * xx[2];
  xx[17] = xx[1];
  xx[18] = xx[1];
  xx[19] = xx[1];
  xx[20] = xx[1];
  xx[21] = xx[3];
  xx[22] = xx[3];
  xx[23] = xx[3];
  xx[1] = xx[17 + (partialType)];
  xx[3] = cos(xx[1]);
  xx[4] = sin(xx[1]);
  xx[1] = sin(xx[16]);
  xx[10] = xx[0];
  xx[11] = xx[2];
  xx[12] = xx[0];
  xx[13] = xx[2];
  xx[14] = xx[0];
  xx[15] = xx[2];
  xx[16] = xx[0];
  xx[0] = xx[10 + (partialType)];
  xx[2] = cos(xx[0]);
  xx[5] = sin(xx[0]);
  xx[0] = xx[3] * xx[5];
  xx[6] = xx[3] * xx[2];
  xx[10] = xx[9] * xx[3];
  xx[11] = - (xx[9] * xx[4]);
  xx[12] = xx[1];
  xx[13] = xx[2] * xx[4] + xx[0] * xx[1];
  xx[14] = xx[6] - xx[1] * xx[5] * xx[4];
  xx[15] = - (xx[9] * xx[5]);
  xx[16] = xx[4] * xx[5] - xx[6] * xx[1];
  xx[17] = xx[0] + xx[2] * xx[1] * xx[4];
  xx[18] = xx[9] * xx[2];
  pm_math_Quaternion_Matrix3x3Ctor_ra(xx + 10, xx + 0);
  state[119] = xx[0];
  state[120] = xx[1];
  state[121] = xx[2];
  state[122] = xx[3];
}

static void projectPartiallyTargetedPos_20_0(const double *origState, int
  partialType, double *state)
{
  boolean_T bb[1];
  int ii[3];
  double xx[24];
  xx[0] = state[127] * state[129];
  xx[1] = state[126] * state[128];
  xx[2] = 2.0;
  xx[3] = (xx[0] + xx[1]) * xx[2];
  xx[4] = fabs(xx[3]) > 1.0 ? atan2(xx[3], 0.0) : asin(xx[3]);
  xx[5] = origState[127] * origState[129];
  xx[6] = origState[126] * origState[128];
  xx[7] = (xx[5] + xx[6]) * xx[2];
  xx[8] = fabs(xx[7]) > 1.0 ? atan2(xx[7], 0.0) : asin(xx[7]);
  xx[9] = xx[4];
  xx[10] = xx[4];
  xx[11] = xx[8];
  xx[12] = xx[8];
  xx[13] = xx[4];
  xx[14] = xx[4];
  xx[15] = xx[8];
  xx[16] = xx[9 + (partialType)];
  xx[9] = cos(xx[16]);
  xx[10] = 0.99999999999999;
  xx[11] = fabs(xx[3]) - xx[10];
  if (xx[11] < 0.0)
    ii[0] = -1;
  else if (xx[11] > 0.0)
    ii[0] = +1;
  else
    ii[0] = 0;
  ii[1] = ii[0];
  if (0 > ii[1])
    ii[1] = 0;
  bb[0] = !(ii[1] == 1);
  xx[3] = state[127] * state[128];
  xx[11] = state[126] * state[129];
  xx[12] = state[126] * state[126];
  xx[13] = 1.0;
  xx[14] = (xx[12] + state[127] * state[127]) * xx[2] - xx[13];
  xx[15] = - (xx[2] * (xx[3] - xx[11]));
  xx[14] = (xx[15] == 0.0 && xx[14] == 0.0) ? 0.0 : atan2(xx[15], xx[14]);
  if (xx[4] < 0.0)
    xx[15] = -1.0;
  else if (xx[4] > 0.0)
    xx[15] = +1.0;
  else
    xx[15] = 0.0;
  xx[4] = (xx[12] + state[129] * state[129]) * xx[2] - xx[13];
  xx[17] = - (xx[2] * (state[128] * state[129] - state[126] * state[127]));
  xx[4] = (xx[17] == 0.0 && xx[4] == 0.0) ? 0.0 : atan2(xx[17], xx[4]);
  xx[12] = 0.5;
  xx[17] = - (xx[2] * (xx[0] - xx[1]) * xx[15]);
  xx[18] = (xx[3] + xx[11]) * xx[2] * xx[15];
  xx[17] = (xx[18] == 0.0 && xx[17] == 0.0) ? 0.0 : atan2(xx[18], xx[17]);
  xx[0] = bb[0] ? xx[4] : xx[12] * xx[17];
  xx[1] = bb[0] ? xx[14] : xx[15] * xx[0];
  xx[3] = fabs(xx[7]) - xx[10];
  if (xx[3] < 0.0)
    ii[0] = -1;
  else if (xx[3] > 0.0)
    ii[0] = +1;
  else
    ii[0] = 0;
  ii[1] = ii[0];
  if (0 > ii[1])
    ii[1] = 0;
  bb[0] = !(ii[1] == 1);
  xx[3] = origState[127] * origState[128];
  xx[4] = origState[126] * origState[129];
  xx[7] = origState[126] * origState[126];
  xx[10] = (xx[7] + origState[127] * origState[127]) * xx[2] - xx[13];
  xx[11] = - (xx[2] * (xx[3] - xx[4]));
  xx[10] = (xx[11] == 0.0 && xx[10] == 0.0) ? 0.0 : atan2(xx[11], xx[10]);
  if (xx[8] < 0.0)
    xx[11] = -1.0;
  else if (xx[8] > 0.0)
    xx[11] = +1.0;
  else
    xx[11] = 0.0;
  xx[8] = (xx[7] + origState[129] * origState[129]) * xx[2] - xx[13];
  xx[14] = - (xx[2] * (origState[128] * origState[129] - origState[126] *
                       origState[127]));
  xx[8] = (xx[14] == 0.0 && xx[8] == 0.0) ? 0.0 : atan2(xx[14], xx[8]);
  xx[7] = - (xx[2] * (xx[5] - xx[6]) * xx[11]);
  xx[13] = (xx[3] + xx[4]) * xx[2] * xx[11];
  xx[7] = (xx[13] == 0.0 && xx[7] == 0.0) ? 0.0 : atan2(xx[13], xx[7]);
  xx[2] = bb[0] ? xx[8] : xx[12] * xx[7];
  xx[3] = bb[0] ? xx[10] : xx[11] * xx[2];
  xx[17] = xx[1];
  xx[18] = xx[1];
  xx[19] = xx[1];
  xx[20] = xx[1];
  xx[21] = xx[3];
  xx[22] = xx[3];
  xx[23] = xx[3];
  xx[1] = xx[17 + (partialType)];
  xx[3] = cos(xx[1]);
  xx[4] = sin(xx[1]);
  xx[1] = sin(xx[16]);
  xx[10] = xx[0];
  xx[11] = xx[2];
  xx[12] = xx[0];
  xx[13] = xx[2];
  xx[14] = xx[0];
  xx[15] = xx[2];
  xx[16] = xx[0];
  xx[0] = xx[10 + (partialType)];
  xx[2] = cos(xx[0]);
  xx[5] = sin(xx[0]);
  xx[0] = xx[3] * xx[5];
  xx[6] = xx[3] * xx[2];
  xx[10] = xx[9] * xx[3];
  xx[11] = - (xx[9] * xx[4]);
  xx[12] = xx[1];
  xx[13] = xx[2] * xx[4] + xx[0] * xx[1];
  xx[14] = xx[6] - xx[1] * xx[5] * xx[4];
  xx[15] = - (xx[9] * xx[5]);
  xx[16] = xx[4] * xx[5] - xx[6] * xx[1];
  xx[17] = xx[0] + xx[2] * xx[1] * xx[4];
  xx[18] = xx[9] * xx[2];
  pm_math_Quaternion_Matrix3x3Ctor_ra(xx + 10, xx + 0);
  state[126] = xx[0];
  state[127] = xx[1];
  state[128] = xx[2];
  state[129] = xx[3];
}

void dempsystest_bjorn_59d2bdba_5_projectPartiallyTargetedPos(const void *mech,
  size_t stageIdx, size_t primIdx, const double *origState, int partialType,
  double *state)
{
  (void) mech;
  (void) stageIdx;
  (void) primIdx;
  (void) origState;
  (void) partialType;
  (void) state;
  switch ((stageIdx * 6 + primIdx))
  {
   case 3:
    projectPartiallyTargetedPos_0_3(origState, partialType, state);
    break;

   case 6:
    projectPartiallyTargetedPos_1_0(origState, partialType, state);
    break;

   case 12:
    projectPartiallyTargetedPos_2_0(origState, partialType, state);
    break;

   case 18:
    projectPartiallyTargetedPos_3_0(origState, partialType, state);
    break;

   case 36:
    projectPartiallyTargetedPos_6_0(origState, partialType, state);
    break;

   case 42:
    projectPartiallyTargetedPos_7_0(origState, partialType, state);
    break;

   case 48:
    projectPartiallyTargetedPos_8_0(origState, partialType, state);
    break;

   case 54:
    projectPartiallyTargetedPos_9_0(origState, partialType, state);
    break;

   case 60:
    projectPartiallyTargetedPos_10_0(origState, partialType, state);
    break;

   case 66:
    projectPartiallyTargetedPos_11_0(origState, partialType, state);
    break;

   case 72:
    projectPartiallyTargetedPos_12_0(origState, partialType, state);
    break;

   case 78:
    projectPartiallyTargetedPos_13_0(origState, partialType, state);
    break;

   case 96:
    projectPartiallyTargetedPos_16_0(origState, partialType, state);
    break;

   case 102:
    projectPartiallyTargetedPos_17_0(origState, partialType, state);
    break;

   case 108:
    projectPartiallyTargetedPos_18_0(origState, partialType, state);
    break;

   case 114:
    projectPartiallyTargetedPos_19_0(origState, partialType, state);
    break;

   case 120:
    projectPartiallyTargetedPos_20_0(origState, partialType, state);
    break;
  }
}

void dempsystest_bjorn_59d2bdba_5_propagateMotion(const void *mech, const
  RuntimeDerivedValuesBundle *rtdv, const double *state, double *motionData)
{
  const double *rtdvd = rtdv->mDoubles.mValues;
  const int *rtdvi = rtdv->mInts.mValues;
  double xx[520];
  (void) mech;
  (void) rtdvd;
  (void) rtdvi;
  xx[0] = state[3];
  xx[1] = state[4];
  xx[2] = state[5];
  xx[3] = state[6];
  xx[4] = 0.9999188709407687;
  xx[5] = 1.729133611399561e-4;
  xx[6] = 0.01273662563857442;
  xx[7] = 2.202511436425175e-6;
  pm_math_Quaternion_composeInverse_ra(xx + 0, xx + 4, xx + 8);
  xx[0] = 9.445556060168147e-5;
  xx[1] = 2.390130654010668e-5;
  xx[2] = xx[0] * xx[10] - xx[1] * xx[11];
  xx[3] = xx[0] * xx[9];
  xx[12] = xx[2];
  xx[13] = - xx[3];
  xx[14] = xx[1] * xx[9];
  pm_math_Vector3_cross_ra(xx + 9, xx + 12, xx + 15);
  xx[12] = 2.0;
  xx[13] = state[0] - (xx[8] * xx[2] + xx[15]) * xx[12] + 1.552;
  xx[2] = state[1] - xx[12] * (xx[16] - xx[8] * xx[3]) - xx[1] + 0.6;
  xx[3] = state[2] - (xx[0] + (xx[1] * xx[8] * xx[9] + xx[17]) * xx[12]);
  xx[14] = - xx[4];
  xx[15] = - xx[6];
  xx[16] = xx[14];
  xx[17] = - xx[5];
  xx[18] = xx[15];
  xx[19] = - xx[7];
  xx[20] = - state[13];
  xx[21] = - state[14];
  xx[22] = - state[15];
  xx[23] = - state[16];
  pm_math_Quaternion_compose_ra(xx + 16, xx + 20, xx + 24);
  xx[16] = 0.0;
  xx[17] = 0.0369355852954221;
  xx[18] = 0.1459657198813009;
  xx[19] = state[20];
  xx[20] = - state[21];
  xx[21] = - state[22];
  xx[22] = - state[23];
  pm_math_Quaternion_compose_ra(xx + 4, xx + 19, xx + 28);
  xx[19] = 7.343000401344803e-3;
  xx[20] = - xx[19];
  xx[21] = 0.06145219684944689;
  xx[22] = - xx[21];
  xx[23] = 0.01771595381192259;
  xx[32] = 0.9998343934839863;
  xx[33] = 0.0181985056119827;
  xx[34] = - (xx[32] * state[27] + xx[33] * state[29]);
  xx[35] = xx[32] * state[28] - xx[33] * state[30];
  xx[36] = xx[32] * state[29] - xx[33] * state[27];
  xx[37] = xx[33] * state[28] + xx[32] * state[30];
  xx[38] = 0.14279;
  xx[39] = 0.8116895113357588;
  xx[40] = 0.2717;
  xx[41] = 0.03913410103116464;
  xx[42] = - xx[39];
  xx[43] = - xx[40];
  xx[44] = - xx[41];
  pm_math_Quaternion_xform_ra(xx + 34, xx + 42, xx + 45);
  xx[48] = xx[38] + xx[45];
  xx[49] = 0.26683;
  xx[50] = xx[46] - xx[49];
  xx[45] = 0.05344999999999999;
  xx[46] = xx[47] - xx[45];
  xx[47] = 0.9691346498545245;
  xx[51] = 0.5;
  xx[52] = xx[51] * state[34];
  xx[53] = cos(xx[52]);
  xx[54] = 0.2458594005622715;
  xx[55] = sin(xx[52]);
  xx[52] = xx[47] * xx[53] - xx[54] * xx[55];
  xx[56] = xx[47] * xx[55] + xx[54] * xx[53];
  xx[57] = 4.475014772496788e-3;
  xx[58] = 0.01763972361731614;
  xx[59] = xx[57] * xx[55] - xx[58] * xx[53];
  xx[60] = xx[57] * xx[53] + xx[58] * xx[55];
  xx[53] = 0.8724478116273718;
  xx[55] = 0.02673029948304496;
  xx[61] = 0.03945399617968662;
  xx[62] = xx[55] * xx[60] - xx[59] * xx[61];
  xx[63] = xx[56];
  xx[64] = xx[59];
  xx[65] = xx[60];
  xx[66] = xx[56] * xx[61];
  xx[67] = xx[56] * xx[55];
  xx[68] = xx[62];
  xx[69] = xx[66];
  xx[70] = - xx[67];
  pm_math_Vector3_cross_ra(xx + 63, xx + 68, xx + 71);
  xx[63] = - (xx[53] + (xx[52] * xx[62] + xx[71]) * xx[12]);
  xx[62] = 0.30545;
  xx[64] = - (xx[62] + xx[12] * (xx[72] + xx[66] * xx[52]) - xx[55]);
  xx[65] = 0.1359107893531998;
  xx[66] = xx[65] - ((xx[73] - xx[67] * xx[52]) * xx[12] - xx[61]);
  xx[67] = xx[51] * state[36];
  xx[68] = cos(xx[67]);
  xx[69] = sin(xx[67]);
  xx[67] = xx[47] * xx[68] + xx[54] * xx[69];
  xx[70] = xx[47] * xx[69] - xx[54] * xx[68];
  xx[47] = xx[58] * xx[68] + xx[57] * xx[69];
  xx[54] = - xx[47];
  xx[71] = xx[58] * xx[69] - xx[57] * xx[68];
  xx[57] = xx[47] * xx[61] - xx[55] * xx[71];
  xx[72] = xx[70];
  xx[73] = xx[54];
  xx[74] = xx[71];
  xx[47] = xx[70] * xx[61];
  xx[58] = xx[70] * xx[55];
  xx[75] = xx[57];
  xx[76] = xx[47];
  xx[77] = xx[58];
  pm_math_Vector3_cross_ra(xx + 72, xx + 75, xx + 78);
  xx[68] = - (xx[53] + (xx[67] * xx[57] + xx[78]) * xx[12]);
  xx[53] = xx[62] - (xx[55] + xx[12] * (xx[79] + xx[47] * xx[67]));
  xx[47] = xx[65] - ((xx[58] * xx[67] + xx[80]) * xx[12] - xx[61]);
  xx[55] = xx[32] * state[38] + xx[33] * state[40];
  xx[57] = xx[32] * state[39] - xx[33] * state[41];
  xx[58] = xx[32] * state[40] - xx[33] * state[38];
  xx[61] = xx[32] * state[41] + xx[33] * state[39];
  xx[72] = xx[55];
  xx[73] = xx[57];
  xx[74] = xx[58];
  xx[75] = xx[61];
  xx[76] = - 0.513;
  xx[77] = 0.27424;
  xx[78] = - 8.0e-3;
  pm_math_Quaternion_xform_ra(xx + 72, xx + 76, xx + 79);
  xx[62] = xx[79] - 0.3579665484632664;
  xx[65] = 0.2646 + xx[80];
  xx[69] = xx[81] - 0.1015240013853662;
  xx[79] = 0.9999999561635808;
  xx[80] = 2.960959925766019e-4;
  xx[81] = xx[79] * state[45] - xx[80] * state[46];
  xx[82] = - xx[81];
  xx[83] = xx[80] * state[45] + xx[79] * state[46];
  xx[84] = xx[79] * state[47] - xx[80] * state[48];
  xx[85] = xx[79] * state[48] + xx[80] * state[47];
  xx[86] = 0.1419467075027625;
  xx[87] = 0.06503381342622938;
  xx[88] = xx[86] * xx[84] - xx[85] * xx[87];
  xx[89] = xx[83] * xx[86];
  xx[90] = xx[83] * xx[87];
  xx[91] = xx[88];
  xx[92] = - xx[89];
  xx[93] = xx[90];
  pm_math_Vector3_cross_ra(xx + 83, xx + 91, xx + 94);
  xx[91] = xx[12] * (xx[94] - xx[88] * xx[81]);
  xx[88] = xx[87] + xx[12] * (xx[95] + xx[81] * xx[89]);
  xx[89] = xx[86] + (xx[96] - xx[81] * xx[90]) * xx[12];
  xx[90] = xx[79] * state[52] + xx[80] * state[53];
  xx[92] = xx[80] * state[52] - xx[79] * state[53];
  xx[93] = xx[80] * state[55] - xx[79] * state[54];
  xx[94] = - (xx[79] * state[55] + xx[80] * state[54]);
  xx[95] = 0.09037053171319968;
  xx[96] = 0.02136831693297856;
  xx[97] = state[59];
  xx[98] = state[60];
  xx[99] = state[61];
  xx[100] = state[62];
  xx[101] = 7.5e-3;
  xx[102] = 0.1764500000000001;
  xx[103] = 0.20773;
  xx[104] = xx[101];
  xx[105] = - xx[102];
  xx[106] = xx[103];
  pm_math_Quaternion_xform_ra(xx + 97, xx + 104, xx + 107);
  xx[110] = xx[32] * state[66] + xx[33] * state[68];
  xx[111] = xx[32] * state[67] - xx[33] * state[69];
  xx[112] = xx[32] * state[68] - xx[33] * state[66];
  xx[113] = xx[32] * state[69] + xx[33] * state[67];
  xx[114] = - 0.5109;
  xx[115] = - 0.284;
  xx[116] = 0.01433000000000001;
  pm_math_Quaternion_xform_ra(xx + 110, xx + 114, xx + 117);
  xx[120] = xx[117] - 0.3651962861772049;
  xx[121] = xx[118] - 0.2296;
  xx[117] = 0.03930618320795053 + xx[119];
  xx[122] = state[73];
  xx[123] = state[74];
  xx[124] = state[75];
  xx[125] = state[76];
  pm_math_Quaternion_xform_ra(xx + 122, xx + 101, xx + 126);
  xx[118] = 2.960959925763799e-4;
  xx[119] = xx[118] * state[81] - xx[79] * state[80];
  xx[129] = xx[118] * state[80] + xx[79] * state[81];
  xx[130] = - xx[129];
  xx[131] = xx[118] * state[83] + xx[79] * state[82];
  xx[132] = - xx[131];
  xx[133] = xx[79] * state[83] - xx[118] * state[82];
  xx[134] = - xx[133];
  xx[135] = 0.0213683169329786;
  xx[136] = 0.09037053171319967;
  xx[137] = xx[135] * xx[131] - xx[133] * xx[136];
  xx[138] = xx[130];
  xx[139] = xx[132];
  xx[140] = xx[134];
  xx[141] = xx[135] * xx[129];
  xx[142] = xx[136] * xx[129];
  xx[143] = xx[137];
  xx[144] = - xx[141];
  xx[145] = xx[142];
  pm_math_Vector3_cross_ra(xx + 138, xx + 143, xx + 146);
  xx[143] = (xx[137] * xx[119] + xx[146]) * xx[12];
  xx[137] = xx[12] * (xx[147] - xx[141] * xx[119]);
  xx[141] = xx[137] - xx[136];
  xx[144] = (xx[142] * xx[119] + xx[148]) * xx[12];
  xx[142] = xx[144] - xx[135];
  xx[145] = xx[118] * state[88] + xx[79] * state[87];
  xx[146] = - (xx[118] * state[87] - xx[79] * state[88]);
  xx[147] = xx[118] * state[90] + xx[79] * state[89];
  xx[148] = - (xx[118] * state[89] - xx[79] * state[90]);
  xx[149] = 0.06503381342622944;
  xx[150] = 0.1419467075027625;
  xx[151] = 0.9996758317937826;
  xx[152] = xx[51] * state[94];
  xx[153] = cos(xx[152]);
  xx[154] = 0.01780577773926495;
  xx[155] = sin(xx[152]);
  xx[152] = xx[151] * xx[153] + xx[154] * xx[155];
  xx[156] = xx[151] * xx[155] - xx[154] * xx[153];
  xx[157] = 0.01819561954822274;
  xx[158] = 3.240922178968028e-4;
  xx[159] = xx[157] * xx[153] + xx[158] * xx[155];
  xx[160] = - xx[159];
  xx[161] = xx[157] * xx[155] - xx[158] * xx[153];
  xx[153] = 0.6786740914416088;
  xx[155] = 5.002590631222067e-3;
  xx[162] = 0.05733821843436025;
  xx[163] = xx[159] * xx[155] + xx[162] * xx[161];
  xx[164] = xx[156];
  xx[165] = xx[160];
  xx[166] = xx[161];
  xx[159] = xx[156] * xx[155];
  xx[167] = xx[156] * xx[162];
  xx[168] = xx[163];
  xx[169] = xx[159];
  xx[170] = - xx[167];
  pm_math_Vector3_cross_ra(xx + 164, xx + 168, xx + 171);
  xx[164] = xx[153] - (xx[152] * xx[163] + xx[171]) * xx[12];
  xx[163] = 0.246;
  xx[165] = xx[163] - (xx[12] * (xx[172] + xx[159] * xx[152]) - xx[162]);
  xx[159] = 0.07278059000796119;
  xx[166] = - (xx[159] + (xx[173] - xx[167] * xx[152]) * xx[12] - xx[155]);
  xx[167] = xx[51] * state[96];
  xx[51] = cos(xx[167]);
  xx[168] = sin(xx[167]);
  xx[167] = xx[151] * xx[51] - xx[154] * xx[168];
  xx[169] = xx[151] * xx[168] + xx[154] * xx[51];
  xx[151] = xx[158] * xx[168] - xx[157] * xx[51];
  xx[154] = xx[158] * xx[51] + xx[157] * xx[168];
  xx[170] = xx[169];
  xx[171] = xx[151];
  xx[172] = xx[154];
  xx[51] = xx[151] * xx[155] + xx[162] * xx[154];
  xx[157] = xx[169] * xx[155];
  xx[158] = xx[169] * xx[162];
  xx[173] = - xx[51];
  xx[174] = xx[157];
  xx[175] = xx[158];
  pm_math_Vector3_cross_ra(xx + 170, xx + 173, xx + 176);
  xx[168] = xx[153] - (xx[176] - xx[167] * xx[51]) * xx[12];
  xx[51] = - (xx[163] + xx[162] + xx[12] * (xx[177] + xx[157] * xx[167]));
  xx[153] = - (xx[159] + (xx[158] * xx[167] + xx[178]) * xx[12] - xx[155]);
  xx[157] = xx[32] * state[98] + xx[33] * state[100];
  xx[158] = xx[32] * state[99] - xx[33] * state[101];
  xx[159] = xx[32] * state[100] - xx[33] * state[98];
  xx[163] = xx[32] * state[101] + xx[33] * state[99];
  xx[170] = xx[157];
  xx[171] = xx[158];
  xx[172] = xx[159];
  xx[173] = xx[163];
  xx[174] = - xx[38];
  xx[175] = - xx[49];
  xx[176] = xx[45];
  pm_math_Quaternion_xform_ra(xx + 170, xx + 174, xx + 177);
  xx[32] = xx[39] + xx[177];
  xx[33] = xx[178] - xx[40];
  xx[38] = xx[41] + xx[179];
  xx[177] = state[105];
  xx[178] = state[106];
  xx[179] = state[107];
  xx[180] = state[108];
  xx[39] = 5.789999999999964e-3;
  xx[40] = 0.17753;
  xx[41] = - 0.15579;
  pm_math_Quaternion_xform_ra(xx + 177, xx + 39, xx + 181);
  xx[184] = state[112];
  xx[185] = state[113];
  xx[186] = state[114];
  xx[187] = state[115];
  xx[45] = 1.729133611397341e-4;
  xx[49] = 2.20251143642801e-6;
  xx[188] = xx[4];
  xx[189] = - xx[45];
  xx[190] = xx[6];
  xx[191] = - xx[49];
  pm_math_Quaternion_composeInverse_ra(xx + 184, xx + 188, xx + 192);
  xx[184] = 7.343000401344803e-3;
  xx[185] = - 0.06145219684944687;
  xx[186] = - 0.01771595381192262;
  pm_math_Quaternion_xform_ra(xx + 192, xx + 184, xx + 196);
  xx[199] = xx[14];
  xx[200] = xx[45];
  xx[201] = xx[15];
  xx[202] = xx[49];
  xx[203] = - state[119];
  xx[204] = - state[120];
  xx[205] = - state[121];
  xx[206] = - state[122];
  pm_math_Quaternion_compose_ra(xx + 199, xx + 203, xx + 207);
  xx[14] = 0.03693558529542217;
  xx[15] = 0.1459657198813009;
  xx[199] = state[126];
  xx[200] = state[127];
  xx[201] = state[128];
  xx[202] = state[129];
  xx[203] = xx[39];
  xx[204] = - xx[40];
  xx[205] = xx[41];
  pm_math_Quaternion_xform_ra(xx + 199, xx + 203, xx + 211);
  pm_math_Quaternion_compose_ra(xx + 8, xx + 28, xx + 214);
  xx[218] = xx[20];
  xx[219] = xx[22];
  xx[220] = xx[23];
  pm_math_Quaternion_xform_ra(xx + 8, xx + 218, xx + 221);
  xx[45] = xx[221] + xx[13];
  xx[49] = xx[222] + xx[2];
  xx[187] = xx[223] + xx[3];
  pm_math_Quaternion_compose_ra(xx + 214, xx + 34, xx + 221);
  xx[225] = xx[48];
  xx[226] = xx[50];
  xx[227] = xx[46];
  pm_math_Quaternion_xform_ra(xx + 214, xx + 225, xx + 228);
  xx[206] = xx[228] + xx[45];
  xx[231] = xx[229] + xx[49];
  xx[228] = xx[230] + xx[187];
  pm_math_Quaternion_compose_ra(xx + 221, xx + 110, xx + 232);
  xx[236] = xx[120];
  xx[237] = xx[121];
  xx[238] = xx[117];
  pm_math_Quaternion_xform_ra(xx + 221, xx + 236, xx + 239);
  xx[229] = xx[239] + xx[206];
  xx[230] = xx[240] + xx[231];
  xx[239] = xx[241] + xx[228];
  xx[240] = xx[119];
  xx[241] = xx[130];
  xx[242] = xx[132];
  xx[243] = xx[134];
  pm_math_Quaternion_compose_ra(xx + 232, xx + 240, xx + 244);
  xx[248] = xx[143];
  xx[249] = xx[141];
  xx[250] = xx[142];
  pm_math_Quaternion_xform_ra(xx + 232, xx + 248, xx + 251);
  pm_math_Quaternion_compose_ra(xx + 110, xx + 240, xx + 254);
  pm_math_Quaternion_xform_ra(xx + 110, xx + 248, xx + 258);
  xx[261] = xx[258] + xx[120];
  xx[262] = xx[259] + xx[121];
  xx[258] = xx[260] + xx[117];
  pm_math_Quaternion_compose_ra(xx + 34, xx + 254, xx + 263);
  xx[267] = xx[261];
  xx[268] = xx[262];
  xx[269] = xx[258];
  pm_math_Quaternion_xform_ra(xx + 34, xx + 267, xx + 270);
  xx[267] = xx[270] + xx[48];
  xx[268] = xx[271] + xx[50];
  xx[269] = xx[272] + xx[46];
  pm_math_Quaternion_compose_ra(xx + 28, xx + 263, xx + 270);
  pm_math_Quaternion_xform_ra(xx + 28, xx + 267, xx + 274);
  pm_math_Quaternion_compose_ra(xx + 221, xx + 72, xx + 277);
  xx[281] = xx[62];
  xx[282] = xx[65];
  xx[283] = xx[69];
  pm_math_Quaternion_xform_ra(xx + 221, xx + 281, xx + 284);
  xx[287] = xx[284] + xx[206];
  xx[288] = xx[285] + xx[231];
  xx[284] = xx[286] + xx[228];
  pm_math_Quaternion_compose_ra(xx + 277, xx + 82, xx + 289);
  xx[293] = xx[91];
  xx[294] = xx[88];
  xx[295] = xx[89];
  pm_math_Quaternion_xform_ra(xx + 277, xx + 293, xx + 296);
  pm_math_Quaternion_compose_ra(xx + 72, xx + 82, xx + 299);
  pm_math_Quaternion_xform_ra(xx + 72, xx + 293, xx + 303);
  xx[285] = xx[303] + xx[62];
  xx[286] = xx[304] + xx[65];
  xx[303] = xx[305] + xx[69];
  pm_math_Quaternion_compose_ra(xx + 34, xx + 299, xx + 304);
  xx[308] = xx[285];
  xx[309] = xx[286];
  xx[310] = xx[303];
  pm_math_Quaternion_xform_ra(xx + 34, xx + 308, xx + 311);
  xx[308] = xx[311] + xx[48];
  xx[309] = xx[312] + xx[50];
  xx[310] = xx[313] + xx[46];
  pm_math_Quaternion_compose_ra(xx + 28, xx + 304, xx + 311);
  pm_math_Quaternion_xform_ra(xx + 28, xx + 308, xx + 315);
  pm_math_Quaternion_compose_ra(xx + 221, xx + 170, xx + 318);
  xx[322] = xx[32];
  xx[323] = xx[33];
  xx[324] = xx[38];
  pm_math_Quaternion_xform_ra(xx + 221, xx + 322, xx + 325);
  xx[328] = xx[325] + xx[206];
  xx[329] = xx[326] + xx[231];
  xx[325] = xx[327] + xx[228];
  pm_math_Quaternion_compose_ra(xx + 318, xx + 192, xx + 330);
  pm_math_Quaternion_xform_ra(xx + 318, xx + 196, xx + 334);
  pm_math_Quaternion_compose_ra(xx + 170, xx + 192, xx + 337);
  pm_math_Quaternion_xform_ra(xx + 170, xx + 196, xx + 341);
  xx[326] = xx[341] + xx[32];
  xx[327] = xx[342] + xx[33];
  xx[341] = xx[343] + xx[38];
  pm_math_Quaternion_compose_ra(xx + 34, xx + 337, xx + 344);
  xx[348] = xx[326];
  xx[349] = xx[327];
  xx[350] = xx[341];
  pm_math_Quaternion_xform_ra(xx + 34, xx + 348, xx + 351);
  xx[348] = xx[351] + xx[48];
  xx[349] = xx[352] + xx[50];
  xx[350] = xx[353] + xx[46];
  pm_math_Quaternion_compose_ra(xx + 28, xx + 344, xx + 351);
  pm_math_Quaternion_xform_ra(xx + 28, xx + 348, xx + 355);
  pm_math_Quaternion_compose_ra(xx + 110, xx + 122, xx + 358);
  pm_math_Quaternion_xform_ra(xx + 110, xx + 126, xx + 362);
  xx[365] = xx[90];
  xx[366] = xx[92];
  xx[367] = xx[93];
  xx[368] = xx[94];
  pm_math_Quaternion_compose_ra(xx + 299, xx + 365, xx + 369);
  xx[373] = xx[96] * xx[301] + xx[95] * xx[302];
  xx[374] = xx[96] * xx[300];
  xx[375] = xx[373];
  xx[376] = - xx[374];
  xx[377] = - (xx[95] * xx[300]);
  pm_math_Vector3_cross_ra(xx + 300, xx + 375, xx + 378);
  xx[375] = (xx[299] * xx[373] + xx[378]) * xx[12] + xx[285];
  xx[373] = xx[12] * (xx[379] - xx[299] * xx[374]) - xx[95] + xx[286];
  xx[374] = xx[96] + (xx[380] - xx[95] * xx[299] * xx[300]) * xx[12] + xx[303];
  pm_math_Quaternion_compose_ra(xx + 369, xx + 97, xx + 376);
  pm_math_Quaternion_xform_ra(xx + 369, xx + 107, xx + 380);
  pm_math_Quaternion_compose_ra(xx + 365, xx + 97, xx + 383);
  pm_math_Quaternion_xform_ra(xx + 365, xx + 107, xx + 387);
  xx[390] = xx[388] - xx[95];
  xx[388] = xx[389] + xx[96];
  pm_math_Quaternion_compose_ra(xx + 82, xx + 383, xx + 391);
  xx[395] = xx[387];
  xx[396] = xx[390];
  xx[397] = xx[388];
  pm_math_Quaternion_xform_ra(xx + 82, xx + 395, xx + 398);
  pm_math_Quaternion_compose_ra(xx + 254, xx + 145, xx + 401);
  xx[389] = xx[150] * xx[256] + xx[149] * xx[257];
  xx[395] = xx[150] * xx[255];
  xx[405] = - xx[389];
  xx[406] = xx[395];
  xx[407] = xx[149] * xx[255];
  pm_math_Vector3_cross_ra(xx + 255, xx + 405, xx + 408);
  pm_math_Quaternion_compose_ra(xx + 240, xx + 145, xx + 411);
  xx[396] = xx[150] * xx[131] + xx[133] * xx[149];
  xx[131] = xx[150] * xx[129];
  xx[133] = xx[149] * xx[129];
  xx[405] = xx[396];
  xx[406] = - xx[131];
  xx[407] = - xx[133];
  pm_math_Vector3_cross_ra(xx + 138, xx + 405, xx + 415);
  pm_math_Quaternion_compose_ra(xx + 82, xx + 365, xx + 418);
  xx[129] = xx[96] * xx[84] + xx[85] * xx[95];
  xx[138] = xx[83] * xx[96];
  xx[139] = xx[83] * xx[95];
  xx[405] = xx[129];
  xx[406] = - xx[138];
  xx[407] = - xx[139];
  pm_math_Vector3_cross_ra(xx + 83, xx + 405, xx + 422);
  pm_math_Quaternion_compose_ra(xx + 337, xx + 207, xx + 425);
  xx[140] = xx[15] * xx[339] + xx[14] * xx[340];
  xx[397] = xx[15] * xx[338];
  xx[405] = - xx[140];
  xx[406] = xx[397];
  xx[407] = xx[14] * xx[338];
  pm_math_Vector3_cross_ra(xx + 338, xx + 405, xx + 429);
  pm_math_Quaternion_compose_ra(xx + 192, xx + 207, xx + 432);
  xx[405] = xx[15] * xx[194] + xx[14] * xx[195];
  xx[406] = xx[15] * xx[193];
  xx[436] = - xx[405];
  xx[437] = xx[406];
  xx[438] = xx[14] * xx[193];
  pm_math_Vector3_cross_ra(xx + 193, xx + 436, xx + 439);
  pm_math_Quaternion_compose_ra(xx + 28, xx + 34, xx + 442);
  pm_math_Quaternion_xform_ra(xx + 28, xx + 225, xx + 436);
  xx[446] = xx[152];
  xx[447] = xx[156];
  xx[448] = xx[160];
  xx[449] = xx[161];
  pm_math_Quaternion_compose_ra(xx + 34, xx + 446, xx + 450);
  pm_math_Quaternion_xform_ra(xx + 34, xx + 164, xx + 454);
  pm_math_Quaternion_compose_ra(xx + 170, xx + 177, xx + 457);
  pm_math_Quaternion_xform_ra(xx + 170, xx + 181, xx + 461);
  xx[464] = state[10];
  xx[465] = state[11];
  xx[466] = state[12];
  pm_math_Quaternion_xform_ra(xx + 4, xx + 464, xx + 467);
  xx[464] = state[7];
  xx[465] = state[8];
  xx[466] = state[9];
  pm_math_Quaternion_inverseXform_ra(xx + 8, xx + 464, xx + 470);
  xx[407] = xx[470] + xx[1] * xx[469] - xx[0] * xx[468];
  xx[464] = xx[471] + xx[0] * xx[467];
  xx[0] = xx[472] - xx[1] * xx[467];
  pm_math_Quaternion_inverseXform_ra(xx + 24, xx + 467, xx + 470);
  xx[473] = xx[17] * xx[469] - xx[18] * xx[468] + xx[407];
  xx[474] = xx[464] + xx[18] * xx[467];
  xx[475] = xx[0] - xx[17] * xx[467];
  pm_math_Quaternion_inverseXform_ra(xx + 24, xx + 473, xx + 476);
  pm_math_Quaternion_inverseXform_ra(xx + 28, xx + 467, xx + 473);
  xx[479] = - state[24];
  xx[480] = - state[25];
  xx[481] = - state[26];
  pm_math_Quaternion_xform_ra(xx + 4, xx + 479, xx + 482);
  pm_math_Quaternion_inverseXform_ra(xx + 28, xx + 482, xx + 4);
  xx[1] = xx[473] + xx[4];
  xx[7] = xx[474] + xx[5];
  xx[4] = xx[475] + xx[6];
  pm_math_Vector3_cross_ra(xx + 467, xx + 218, xx + 473);
  xx[218] = xx[473] + xx[407];
  xx[219] = xx[474] + xx[464];
  xx[220] = xx[475] + xx[0];
  pm_math_Quaternion_inverseXform_ra(xx + 28, xx + 218, xx + 473);
  xx[218] = xx[1];
  xx[219] = xx[7];
  xx[220] = xx[4];
  pm_math_Quaternion_inverseXform_ra(xx + 34, xx + 218, xx + 479);
  xx[482] = - state[31];
  xx[483] = - state[32];
  xx[484] = - state[33];
  pm_math_Quaternion_inverseXform_ra(xx + 34, xx + 482, xx + 485);
  xx[5] = xx[479] + xx[485];
  xx[6] = xx[480] + xx[486];
  xx[465] = xx[481] + xx[487];
  pm_math_Vector3_cross_ra(xx + 218, xx + 225, xx + 479);
  xx[225] = xx[479] + xx[473];
  xx[226] = xx[480] + xx[474];
  xx[227] = xx[481] + xx[475];
  pm_math_Quaternion_inverseXform_ra(xx + 34, xx + 225, xx + 479);
  pm_math_Vector3_cross_ra(xx + 485, xx + 42, xx + 225);
  xx[42] = xx[479] + xx[225];
  xx[43] = xx[480] + xx[226];
  xx[44] = xx[481] + xx[227];
  xx[479] = xx[52];
  xx[480] = xx[56];
  xx[481] = xx[59];
  xx[482] = xx[60];
  xx[225] = xx[5];
  xx[226] = xx[6];
  xx[227] = xx[465];
  pm_math_Quaternion_inverseXform_ra(xx + 479, xx + 225, xx + 483);
  xx[486] = xx[63];
  xx[487] = xx[64];
  xx[488] = xx[66];
  pm_math_Vector3_cross_ra(xx + 225, xx + 486, xx + 489);
  xx[486] = xx[489] + xx[42];
  xx[487] = xx[490] + xx[43];
  xx[488] = xx[491] + xx[44];
  pm_math_Quaternion_inverseXform_ra(xx + 479, xx + 486, xx + 489);
  xx[466] = 0.03945399617968662;
  xx[479] = 0.02673029948304497;
  xx[492] = xx[67];
  xx[493] = xx[70];
  xx[494] = xx[54];
  xx[495] = xx[71];
  pm_math_Quaternion_inverseXform_ra(xx + 492, xx + 225, xx + 480);
  xx[486] = xx[68];
  xx[487] = xx[53];
  xx[488] = xx[47];
  pm_math_Vector3_cross_ra(xx + 225, xx + 486, xx + 496);
  xx[486] = xx[496] + xx[42];
  xx[487] = xx[497] + xx[43];
  xx[488] = xx[498] + xx[44];
  pm_math_Quaternion_inverseXform_ra(xx + 492, xx + 486, xx + 496);
  pm_math_Quaternion_inverseXform_ra(xx + 72, xx + 225, xx + 486);
  xx[492] = xx[486] + state[42];
  xx[493] = xx[487] + state[43];
  xx[486] = xx[488] + state[44];
  pm_math_Vector3_cross_ra(xx + 225, xx + 281, xx + 499);
  xx[281] = xx[499] + xx[42];
  xx[282] = xx[500] + xx[43];
  xx[283] = xx[501] + xx[44];
  pm_math_Quaternion_inverseXform_ra(xx + 72, xx + 281, xx + 499);
  xx[72] = state[42];
  xx[73] = state[43];
  xx[74] = state[44];
  pm_math_Vector3_cross_ra(xx + 72, xx + 76, xx + 281);
  xx[72] = xx[499] + xx[281];
  xx[73] = xx[500] + xx[282];
  xx[74] = xx[501] + xx[283];
  xx[75] = xx[492];
  xx[76] = xx[493];
  xx[77] = xx[486];
  pm_math_Quaternion_inverseXform_ra(xx + 82, xx + 75, xx + 281);
  xx[499] = - state[49];
  xx[500] = - state[50];
  xx[501] = - state[51];
  pm_math_Quaternion_inverseXform_ra(xx + 82, xx + 499, xx + 502);
  xx[78] = xx[281] + xx[502];
  xx[487] = xx[282] + xx[503];
  xx[281] = xx[283] + xx[504];
  pm_math_Vector3_cross_ra(xx + 75, xx + 293, xx + 499);
  xx[75] = xx[499] + xx[72];
  xx[76] = xx[500] + xx[73];
  xx[77] = xx[501] + xx[74];
  pm_math_Quaternion_inverseXform_ra(xx + 82, xx + 75, xx + 293);
  xx[75] = xx[293] + xx[86] * xx[503] - xx[87] * xx[504];
  xx[76] = xx[294] - xx[86] * xx[502];
  xx[77] = xx[295] + xx[87] * xx[502];
  xx[293] = xx[78];
  xx[294] = xx[487];
  xx[295] = xx[281];
  pm_math_Quaternion_inverseXform_ra(xx + 365, xx + 293, xx + 499);
  xx[86] = xx[80] * state[58];
  xx[87] = xx[80] * state[57];
  xx[293] = - state[56];
  xx[294] = (xx[79] * xx[86] + xx[80] * xx[87]) * xx[12] - state[57];
  xx[295] = xx[12] * (xx[80] * xx[86] - xx[79] * xx[87]) - state[58];
  pm_math_Quaternion_inverseXform_ra(xx + 365, xx + 293, xx + 502);
  xx[80] = xx[499] + xx[502];
  xx[86] = xx[500] + xx[503];
  xx[87] = xx[501] + xx[504];
  xx[293] = xx[487] * xx[96] + xx[281] * xx[95] + xx[75];
  xx[294] = xx[76] - xx[78] * xx[96];
  xx[295] = xx[77] - xx[78] * xx[95];
  pm_math_Quaternion_inverseXform_ra(xx + 365, xx + 293, xx + 499);
  xx[293] = xx[80];
  xx[294] = xx[86];
  xx[295] = xx[87];
  pm_math_Quaternion_inverseXform_ra(xx + 97, xx + 293, xx + 365);
  pm_math_Vector3_cross_ra(xx + 293, xx + 107, xx + 502);
  xx[293] = xx[502] + xx[499];
  xx[294] = xx[503] + xx[500];
  xx[295] = xx[504] + xx[501];
  pm_math_Quaternion_inverseXform_ra(xx + 97, xx + 293, xx + 502);
  xx[97] = state[63];
  xx[98] = state[64];
  xx[99] = state[65];
  pm_math_Vector3_cross_ra(xx + 97, xx + 104, xx + 293);
  pm_math_Quaternion_inverseXform_ra(xx + 110, xx + 225, xx + 97);
  xx[100] = xx[97] + state[70];
  xx[104] = xx[98] + state[71];
  xx[97] = xx[99] + state[72];
  pm_math_Vector3_cross_ra(xx + 225, xx + 236, xx + 505);
  xx[236] = xx[505] + xx[42];
  xx[237] = xx[506] + xx[43];
  xx[238] = xx[507] + xx[44];
  pm_math_Quaternion_inverseXform_ra(xx + 110, xx + 236, xx + 505);
  xx[236] = state[70];
  xx[237] = state[71];
  xx[238] = state[72];
  pm_math_Vector3_cross_ra(xx + 236, xx + 114, xx + 508);
  xx[98] = xx[505] + xx[508];
  xx[99] = xx[506] + xx[509];
  xx[105] = xx[507] + xx[510];
  xx[114] = xx[100];
  xx[115] = xx[104];
  xx[116] = xx[97];
  pm_math_Quaternion_inverseXform_ra(xx + 122, xx + 114, xx + 236);
  pm_math_Vector3_cross_ra(xx + 114, xx + 126, xx + 505);
  xx[508] = xx[505] + xx[98];
  xx[509] = xx[506] + xx[99];
  xx[510] = xx[507] + xx[105];
  pm_math_Quaternion_inverseXform_ra(xx + 122, xx + 508, xx + 505);
  xx[122] = state[77];
  xx[123] = state[78];
  xx[124] = state[79];
  pm_math_Vector3_cross_ra(xx + 122, xx + 101, xx + 508);
  pm_math_Quaternion_inverseXform_ra(xx + 240, xx + 114, xx + 101);
  xx[106] = xx[101] + state[84];
  xx[122] = xx[118] * state[85];
  xx[123] = xx[118] * state[86];
  xx[124] = state[85] - (xx[118] * xx[122] - xx[79] * xx[123]) * xx[12];
  xx[125] = xx[102] + xx[124];
  xx[101] = state[86] - xx[12] * (xx[79] * xx[122] + xx[118] * xx[123]);
  xx[79] = xx[103] + xx[101];
  pm_math_Vector3_cross_ra(xx + 114, xx + 248, xx + 511);
  xx[114] = xx[511] + xx[98];
  xx[115] = xx[512] + xx[99];
  xx[116] = xx[513] + xx[105];
  pm_math_Quaternion_inverseXform_ra(xx + 240, xx + 114, xx + 248);
  xx[102] = xx[248] + xx[101] * xx[136] - xx[135] * xx[124];
  xx[101] = xx[249] + xx[135] * state[84];
  xx[103] = xx[250] - xx[136] * state[84];
  xx[114] = xx[106];
  xx[115] = xx[125];
  xx[116] = xx[79];
  pm_math_Quaternion_inverseXform_ra(xx + 145, xx + 114, xx + 122);
  xx[114] = xx[102] - (xx[125] * xx[150] + xx[79] * xx[149]);
  xx[115] = xx[101] + xx[106] * xx[150];
  xx[116] = xx[106] * xx[149] + xx[103];
  pm_math_Quaternion_inverseXform_ra(xx + 145, xx + 114, xx + 240);
  pm_math_Quaternion_inverseXform_ra(xx + 446, xx + 225, xx + 114);
  pm_math_Vector3_cross_ra(xx + 225, xx + 164, xx + 248);
  xx[511] = xx[248] + xx[42];
  xx[512] = xx[249] + xx[43];
  xx[513] = xx[250] + xx[44];
  pm_math_Quaternion_inverseXform_ra(xx + 446, xx + 511, xx + 248);
  xx[446] = xx[167];
  xx[447] = xx[169];
  xx[448] = xx[151];
  xx[449] = xx[154];
  pm_math_Quaternion_inverseXform_ra(xx + 446, xx + 225, xx + 511);
  xx[514] = xx[168];
  xx[515] = xx[51];
  xx[516] = xx[153];
  pm_math_Vector3_cross_ra(xx + 225, xx + 514, xx + 517);
  xx[514] = xx[517] + xx[42];
  xx[515] = xx[518] + xx[43];
  xx[516] = xx[519] + xx[44];
  pm_math_Quaternion_inverseXform_ra(xx + 446, xx + 514, xx + 517);
  pm_math_Quaternion_inverseXform_ra(xx + 170, xx + 225, xx + 446);
  xx[118] = xx[446] + state[102];
  xx[135] = xx[447] + state[103];
  xx[136] = xx[448] + state[104];
  pm_math_Vector3_cross_ra(xx + 225, xx + 322, xx + 446);
  xx[225] = xx[446] + xx[42];
  xx[226] = xx[447] + xx[43];
  xx[227] = xx[448] + xx[44];
  pm_math_Quaternion_inverseXform_ra(xx + 170, xx + 225, xx + 322);
  xx[170] = state[102];
  xx[171] = state[103];
  xx[172] = state[104];
  pm_math_Vector3_cross_ra(xx + 170, xx + 174, xx + 225);
  xx[170] = xx[322] + xx[225];
  xx[171] = xx[323] + xx[226];
  xx[172] = xx[324] + xx[227];
  xx[173] = xx[118];
  xx[174] = xx[135];
  xx[175] = xx[136];
  pm_math_Quaternion_inverseXform_ra(xx + 177, xx + 173, xx + 225);
  pm_math_Vector3_cross_ra(xx + 173, xx + 181, xx + 322);
  xx[446] = xx[322] + xx[170];
  xx[447] = xx[323] + xx[171];
  xx[448] = xx[324] + xx[172];
  pm_math_Quaternion_inverseXform_ra(xx + 177, xx + 446, xx + 322);
  xx[176] = state[109];
  xx[177] = state[110];
  xx[178] = state[111];
  pm_math_Vector3_cross_ra(xx + 176, xx + 39, xx + 446);
  pm_math_Quaternion_inverseXform_ra(xx + 192, xx + 173, xx + 39);
  xx[176] = state[116];
  xx[177] = state[117];
  xx[178] = state[118];
  pm_math_Quaternion_xform_ra(xx + 188, xx + 176, xx + 514);
  xx[176] = xx[39] + xx[514];
  xx[177] = xx[40] + xx[515];
  xx[39] = xx[41] + xx[516];
  pm_math_Vector3_cross_ra(xx + 173, xx + 196, xx + 178);
  xx[173] = xx[178] + xx[170];
  xx[174] = xx[179] + xx[171];
  xx[175] = xx[180] + xx[172];
  pm_math_Quaternion_inverseXform_ra(xx + 192, xx + 173, xx + 178);
  pm_math_Vector3_cross_ra(xx + 514, xx + 184, xx + 173);
  xx[40] = xx[178] + xx[173];
  xx[41] = xx[179] + xx[174];
  xx[173] = xx[180] + xx[175];
  xx[178] = xx[176];
  xx[179] = xx[177];
  xx[180] = xx[39];
  pm_math_Quaternion_inverseXform_ra(xx + 207, xx + 178, xx + 184);
  xx[178] = xx[40] - (xx[177] * xx[15] + xx[39] * xx[14]);
  xx[179] = xx[41] + xx[176] * xx[15];
  xx[180] = xx[176] * xx[14] + xx[173];
  pm_math_Quaternion_inverseXform_ra(xx + 207, xx + 178, xx + 188);
  pm_math_Quaternion_inverseXform_ra(xx + 199, xx + 218, xx + 178);
  pm_math_Vector3_cross_ra(xx + 218, xx + 211, xx + 514);
  xx[218] = xx[514] + xx[473];
  xx[219] = xx[515] + xx[474];
  xx[220] = xx[516] + xx[475];
  pm_math_Quaternion_inverseXform_ra(xx + 199, xx + 218, xx + 514);
  xx[199] = state[130];
  xx[200] = state[131];
  xx[201] = state[132];
  pm_math_Vector3_cross_ra(xx + 199, xx + 203, xx + 218);
  motionData[0] = xx[8];
  motionData[1] = xx[9];
  motionData[2] = xx[10];
  motionData[3] = xx[11];
  motionData[4] = xx[13];
  motionData[5] = xx[2];
  motionData[6] = xx[3];
  motionData[7] = xx[24];
  motionData[8] = xx[25];
  motionData[9] = xx[26];
  motionData[10] = xx[27];
  motionData[11] = xx[16];
  motionData[12] = - xx[17];
  motionData[13] = - xx[18];
  motionData[14] = xx[28];
  motionData[15] = xx[29];
  motionData[16] = xx[30];
  motionData[17] = xx[31];
  motionData[18] = xx[20];
  motionData[19] = xx[22];
  motionData[20] = xx[23];
  motionData[21] = xx[34];
  motionData[22] = xx[35];
  motionData[23] = xx[36];
  motionData[24] = xx[37];
  motionData[25] = xx[48];
  motionData[26] = xx[50];
  motionData[27] = xx[46];
  motionData[28] = xx[52];
  motionData[29] = xx[56];
  motionData[30] = xx[59];
  motionData[31] = xx[60];
  motionData[32] = xx[63];
  motionData[33] = xx[64];
  motionData[34] = xx[66];
  motionData[35] = xx[67];
  motionData[36] = xx[70];
  motionData[37] = xx[54];
  motionData[38] = xx[71];
  motionData[39] = xx[68];
  motionData[40] = xx[53];
  motionData[41] = xx[47];
  motionData[42] = xx[55];
  motionData[43] = xx[57];
  motionData[44] = xx[58];
  motionData[45] = xx[61];
  motionData[46] = xx[62];
  motionData[47] = xx[65];
  motionData[48] = xx[69];
  motionData[49] = xx[82];
  motionData[50] = xx[83];
  motionData[51] = xx[84];
  motionData[52] = xx[85];
  motionData[53] = xx[91];
  motionData[54] = xx[88];
  motionData[55] = xx[89];
  motionData[56] = xx[90];
  motionData[57] = xx[92];
  motionData[58] = xx[93];
  motionData[59] = xx[94];
  motionData[60] = xx[16];
  motionData[61] = - xx[95];
  motionData[62] = xx[96];
  motionData[63] = state[59];
  motionData[64] = state[60];
  motionData[65] = state[61];
  motionData[66] = state[62];
  motionData[67] = xx[107];
  motionData[68] = xx[108];
  motionData[69] = xx[109];
  motionData[70] = xx[110];
  motionData[71] = xx[111];
  motionData[72] = xx[112];
  motionData[73] = xx[113];
  motionData[74] = xx[120];
  motionData[75] = xx[121];
  motionData[76] = xx[117];
  motionData[77] = state[73];
  motionData[78] = state[74];
  motionData[79] = state[75];
  motionData[80] = state[76];
  motionData[81] = xx[126];
  motionData[82] = xx[127];
  motionData[83] = xx[128];
  motionData[84] = xx[119];
  motionData[85] = xx[130];
  motionData[86] = xx[132];
  motionData[87] = xx[134];
  motionData[88] = xx[143];
  motionData[89] = xx[141];
  motionData[90] = xx[142];
  motionData[91] = xx[145];
  motionData[92] = xx[146];
  motionData[93] = xx[147];
  motionData[94] = xx[148];
  motionData[95] = xx[16];
  motionData[96] = xx[149];
  motionData[97] = - xx[150];
  motionData[98] = xx[152];
  motionData[99] = xx[156];
  motionData[100] = xx[160];
  motionData[101] = xx[161];
  motionData[102] = xx[164];
  motionData[103] = xx[165];
  motionData[104] = xx[166];
  motionData[105] = xx[167];
  motionData[106] = xx[169];
  motionData[107] = xx[151];
  motionData[108] = xx[154];
  motionData[109] = xx[168];
  motionData[110] = xx[51];
  motionData[111] = xx[153];
  motionData[112] = xx[157];
  motionData[113] = xx[158];
  motionData[114] = xx[159];
  motionData[115] = xx[163];
  motionData[116] = xx[32];
  motionData[117] = xx[33];
  motionData[118] = xx[38];
  motionData[119] = state[105];
  motionData[120] = state[106];
  motionData[121] = state[107];
  motionData[122] = state[108];
  motionData[123] = xx[181];
  motionData[124] = xx[182];
  motionData[125] = xx[183];
  motionData[126] = xx[192];
  motionData[127] = xx[193];
  motionData[128] = xx[194];
  motionData[129] = xx[195];
  motionData[130] = xx[196];
  motionData[131] = xx[197];
  motionData[132] = xx[198];
  motionData[133] = xx[207];
  motionData[134] = xx[208];
  motionData[135] = xx[209];
  motionData[136] = xx[210];
  motionData[137] = xx[16];
  motionData[138] = xx[14];
  motionData[139] = - xx[15];
  motionData[140] = state[126];
  motionData[141] = state[127];
  motionData[142] = state[128];
  motionData[143] = state[129];
  motionData[144] = xx[211];
  motionData[145] = xx[212];
  motionData[146] = xx[213];
  motionData[147] = xx[214];
  motionData[148] = xx[215];
  motionData[149] = xx[216];
  motionData[150] = xx[217];
  motionData[151] = xx[45];
  motionData[152] = xx[49];
  motionData[153] = xx[187];
  motionData[154] = xx[221];
  motionData[155] = xx[222];
  motionData[156] = xx[223];
  motionData[157] = xx[224];
  motionData[158] = xx[206];
  motionData[159] = xx[231];
  motionData[160] = xx[228];
  motionData[161] = xx[232];
  motionData[162] = xx[233];
  motionData[163] = xx[234];
  motionData[164] = xx[235];
  motionData[165] = xx[229];
  motionData[166] = xx[230];
  motionData[167] = xx[239];
  motionData[168] = xx[244];
  motionData[169] = xx[245];
  motionData[170] = xx[246];
  motionData[171] = xx[247];
  motionData[172] = xx[251] + xx[229];
  motionData[173] = xx[252] + xx[230];
  motionData[174] = xx[253] + xx[239];
  motionData[175] = xx[254];
  motionData[176] = xx[255];
  motionData[177] = xx[256];
  motionData[178] = xx[257];
  motionData[179] = xx[261];
  motionData[180] = xx[262];
  motionData[181] = xx[258];
  motionData[182] = xx[263];
  motionData[183] = xx[264];
  motionData[184] = xx[265];
  motionData[185] = xx[266];
  motionData[186] = xx[267];
  motionData[187] = xx[268];
  motionData[188] = xx[269];
  motionData[189] = xx[270];
  motionData[190] = xx[271];
  motionData[191] = xx[272];
  motionData[192] = xx[273];
  motionData[193] = xx[274] - xx[19];
  motionData[194] = xx[275] - xx[21];
  motionData[195] = xx[276] + xx[23];
  motionData[196] = xx[277];
  motionData[197] = xx[278];
  motionData[198] = xx[279];
  motionData[199] = xx[280];
  motionData[200] = xx[287];
  motionData[201] = xx[288];
  motionData[202] = xx[284];
  motionData[203] = xx[289];
  motionData[204] = xx[290];
  motionData[205] = xx[291];
  motionData[206] = xx[292];
  motionData[207] = xx[296] + xx[287];
  motionData[208] = xx[297] + xx[288];
  motionData[209] = xx[298] + xx[284];
  motionData[210] = xx[299];
  motionData[211] = xx[300];
  motionData[212] = xx[301];
  motionData[213] = xx[302];
  motionData[214] = xx[285];
  motionData[215] = xx[286];
  motionData[216] = xx[303];
  motionData[217] = xx[304];
  motionData[218] = xx[305];
  motionData[219] = xx[306];
  motionData[220] = xx[307];
  motionData[221] = xx[308];
  motionData[222] = xx[309];
  motionData[223] = xx[310];
  motionData[224] = xx[311];
  motionData[225] = xx[312];
  motionData[226] = xx[313];
  motionData[227] = xx[314];
  motionData[228] = xx[315] - xx[19];
  motionData[229] = xx[316] - xx[21];
  motionData[230] = xx[317] + xx[23];
  motionData[231] = xx[318];
  motionData[232] = xx[319];
  motionData[233] = xx[320];
  motionData[234] = xx[321];
  motionData[235] = xx[328];
  motionData[236] = xx[329];
  motionData[237] = xx[325];
  motionData[238] = xx[330];
  motionData[239] = xx[331];
  motionData[240] = xx[332];
  motionData[241] = xx[333];
  motionData[242] = xx[334] + xx[328];
  motionData[243] = xx[335] + xx[329];
  motionData[244] = xx[336] + xx[325];
  motionData[245] = xx[337];
  motionData[246] = xx[338];
  motionData[247] = xx[339];
  motionData[248] = xx[340];
  motionData[249] = xx[326];
  motionData[250] = xx[327];
  motionData[251] = xx[341];
  motionData[252] = xx[344];
  motionData[253] = xx[345];
  motionData[254] = xx[346];
  motionData[255] = xx[347];
  motionData[256] = xx[348];
  motionData[257] = xx[349];
  motionData[258] = xx[350];
  motionData[259] = xx[351];
  motionData[260] = xx[352];
  motionData[261] = xx[353];
  motionData[262] = xx[354];
  motionData[263] = xx[355] - xx[19];
  motionData[264] = xx[356] - xx[21];
  motionData[265] = xx[357] + xx[23];
  motionData[266] = xx[358];
  motionData[267] = xx[359];
  motionData[268] = xx[360];
  motionData[269] = xx[361];
  motionData[270] = xx[362] + xx[120];
  motionData[271] = xx[363] + xx[121];
  motionData[272] = xx[364] + xx[117];
  motionData[273] = xx[369];
  motionData[274] = xx[370];
  motionData[275] = xx[371];
  motionData[276] = xx[372];
  motionData[277] = xx[375];
  motionData[278] = xx[373];
  motionData[279] = xx[374];
  motionData[280] = xx[376];
  motionData[281] = xx[377];
  motionData[282] = xx[378];
  motionData[283] = xx[379];
  motionData[284] = xx[380] + xx[375];
  motionData[285] = xx[381] + xx[373];
  motionData[286] = xx[382] + xx[374];
  motionData[287] = xx[383];
  motionData[288] = xx[384];
  motionData[289] = xx[385];
  motionData[290] = xx[386];
  motionData[291] = xx[387];
  motionData[292] = xx[390];
  motionData[293] = xx[388];
  motionData[294] = xx[391];
  motionData[295] = xx[392];
  motionData[296] = xx[393];
  motionData[297] = xx[394];
  motionData[298] = xx[398] + xx[91];
  motionData[299] = xx[399] + xx[88];
  motionData[300] = xx[400] + xx[89];
  motionData[301] = xx[401];
  motionData[302] = xx[402];
  motionData[303] = xx[403];
  motionData[304] = xx[404];
  motionData[305] = (xx[408] - xx[254] * xx[389]) * xx[12] + xx[261];
  motionData[306] = xx[12] * (xx[409] + xx[254] * xx[395]) + xx[259] + xx[121] +
    xx[149];
  motionData[307] = (xx[149] * xx[254] * xx[255] + xx[410]) * xx[12] + xx[260] +
    xx[117] - xx[150];
  motionData[308] = xx[411];
  motionData[309] = xx[412];
  motionData[310] = xx[413];
  motionData[311] = xx[414];
  motionData[312] = (xx[396] * xx[119] + xx[415]) * xx[12] + xx[143];
  motionData[313] = xx[12] * (xx[416] - xx[131] * xx[119]) + xx[137] -
    0.02533671828697023;
  motionData[314] = (xx[417] - xx[133] * xx[119]) * xx[12] + xx[144] -
    0.1633150244357411;
  motionData[315] = xx[418];
  motionData[316] = xx[419];
  motionData[317] = xx[420];
  motionData[318] = xx[421];
  motionData[319] = xx[12] * (xx[422] - xx[129] * xx[81]) + xx[91];
  motionData[320] = xx[12] * (xx[423] + xx[81] * xx[138]) - xx[95] + xx[88];
  motionData[321] = xx[96] + (xx[81] * xx[139] + xx[424]) * xx[12] + xx[89];
  motionData[322] = xx[425];
  motionData[323] = xx[426];
  motionData[324] = xx[427];
  motionData[325] = xx[428];
  motionData[326] = (xx[429] - xx[337] * xx[140]) * xx[12] + xx[326];
  motionData[327] = xx[12] * (xx[430] + xx[337] * xx[397]) + xx[342] + xx[33] +
    xx[14];
  motionData[328] = (xx[14] * xx[337] * xx[338] + xx[431]) * xx[12] + xx[343] +
    xx[38] - xx[15];
  motionData[329] = xx[432];
  motionData[330] = xx[433];
  motionData[331] = xx[434];
  motionData[332] = xx[435];
  motionData[333] = (xx[439] - xx[192] * xx[405]) * xx[12] + xx[196];
  motionData[334] = xx[12] * (xx[440] + xx[192] * xx[406]) + xx[197] + xx[14];
  motionData[335] = (xx[14] * xx[192] * xx[193] + xx[441]) * xx[12] + xx[198] -
    xx[15];
  motionData[336] = xx[442];
  motionData[337] = xx[443];
  motionData[338] = xx[444];
  motionData[339] = xx[445];
  motionData[340] = xx[436] - xx[19];
  motionData[341] = xx[437] - xx[21];
  motionData[342] = xx[438] + xx[23];
  motionData[343] = xx[450];
  motionData[344] = xx[451];
  motionData[345] = xx[452];
  motionData[346] = xx[453];
  motionData[347] = xx[454] + xx[48];
  motionData[348] = xx[455] + xx[50];
  motionData[349] = xx[456] + xx[46];
  motionData[350] = xx[457];
  motionData[351] = xx[458];
  motionData[352] = xx[459];
  motionData[353] = xx[460];
  motionData[354] = xx[461] + xx[32];
  motionData[355] = xx[462] + xx[33];
  motionData[356] = xx[463] + xx[38];
  motionData[357] = xx[467];
  motionData[358] = xx[468];
  motionData[359] = xx[469];
  motionData[360] = xx[407];
  motionData[361] = xx[464];
  motionData[362] = xx[0];
  motionData[363] = xx[470] + state[17];
  motionData[364] = xx[471] + state[18];
  motionData[365] = xx[472] + state[19];
  motionData[366] = xx[476];
  motionData[367] = xx[477];
  motionData[368] = xx[478];
  motionData[369] = xx[1];
  motionData[370] = xx[7];
  motionData[371] = xx[4];
  motionData[372] = xx[473];
  motionData[373] = xx[474];
  motionData[374] = xx[475];
  motionData[375] = xx[5];
  motionData[376] = xx[6];
  motionData[377] = xx[465];
  motionData[378] = xx[42];
  motionData[379] = xx[43];
  motionData[380] = xx[44];
  motionData[381] = xx[483] + state[35];
  motionData[382] = xx[484];
  motionData[383] = xx[485];
  motionData[384] = xx[489];
  motionData[385] = xx[490] - xx[466] * state[35];
  motionData[386] = xx[491] + xx[479] * state[35];
  motionData[387] = xx[480] + state[37];
  motionData[388] = xx[481];
  motionData[389] = xx[482];
  motionData[390] = xx[496];
  motionData[391] = xx[497] - xx[466] * state[37];
  motionData[392] = xx[498] - xx[479] * state[37];
  motionData[393] = xx[492];
  motionData[394] = xx[493];
  motionData[395] = xx[486];
  motionData[396] = xx[72];
  motionData[397] = xx[73];
  motionData[398] = xx[74];
  motionData[399] = xx[78];
  motionData[400] = xx[487];
  motionData[401] = xx[281];
  motionData[402] = xx[75];
  motionData[403] = xx[76];
  motionData[404] = xx[77];
  motionData[405] = xx[80];
  motionData[406] = xx[86];
  motionData[407] = xx[87];
  motionData[408] = xx[499];
  motionData[409] = xx[500];
  motionData[410] = xx[501];
  motionData[411] = xx[365] + state[63];
  motionData[412] = xx[366] + state[64];
  motionData[413] = xx[367] + state[65];
  motionData[414] = xx[502] + xx[293];
  motionData[415] = xx[503] + xx[294];
  motionData[416] = xx[504] + xx[295];
  motionData[417] = xx[100];
  motionData[418] = xx[104];
  motionData[419] = xx[97];
  motionData[420] = xx[98];
  motionData[421] = xx[99];
  motionData[422] = xx[105];
  motionData[423] = xx[236] + state[77];
  motionData[424] = xx[237] + state[78];
  motionData[425] = xx[238] + state[79];
  motionData[426] = xx[505] + xx[508];
  motionData[427] = xx[506] + xx[509];
  motionData[428] = xx[507] + xx[510];
  motionData[429] = xx[106];
  motionData[430] = xx[125];
  motionData[431] = xx[79];
  motionData[432] = xx[102];
  motionData[433] = xx[101];
  motionData[434] = xx[103];
  motionData[435] = xx[122] + state[91];
  motionData[436] = xx[123] + state[92];
  motionData[437] = xx[124] + state[93];
  motionData[438] = xx[240];
  motionData[439] = xx[241];
  motionData[440] = xx[242];
  motionData[441] = xx[114] + state[95];
  motionData[442] = xx[115];
  motionData[443] = xx[116];
  motionData[444] = xx[248];
  motionData[445] = xx[249] - xx[155] * state[95];
  motionData[446] = xx[250] + xx[162] * state[95];
  motionData[447] = xx[511] + state[97];
  motionData[448] = xx[512];
  motionData[449] = xx[513];
  motionData[450] = xx[517];
  motionData[451] = xx[518] - xx[155] * state[97];
  motionData[452] = xx[519] - xx[162] * state[97];
  motionData[453] = xx[118];
  motionData[454] = xx[135];
  motionData[455] = xx[136];
  motionData[456] = xx[170];
  motionData[457] = xx[171];
  motionData[458] = xx[172];
  motionData[459] = xx[225] + state[109];
  motionData[460] = xx[226] + state[110];
  motionData[461] = xx[227] + state[111];
  motionData[462] = xx[322] + xx[446];
  motionData[463] = xx[323] + xx[447];
  motionData[464] = xx[324] + xx[448];
  motionData[465] = xx[176];
  motionData[466] = xx[177];
  motionData[467] = xx[39];
  motionData[468] = xx[40];
  motionData[469] = xx[41];
  motionData[470] = xx[173];
  motionData[471] = xx[184] + state[123];
  motionData[472] = xx[185] + state[124];
  motionData[473] = xx[186] + state[125];
  motionData[474] = xx[188];
  motionData[475] = xx[189];
  motionData[476] = xx[190];
  motionData[477] = xx[178] + state[130];
  motionData[478] = xx[179] + state[131];
  motionData[479] = xx[180] + state[132];
  motionData[480] = xx[514] + xx[218];
  motionData[481] = xx[515] + xx[219];
  motionData[482] = xx[516] + xx[220];
}

static size_t computeAssemblyError_0(const RuntimeDerivedValuesBundle *rtdv,
  const int *modeVector, const double *motionData, double *error)
{
  const double *rtdvd = rtdv->mDoubles.mValues;
  const int *rtdvi = rtdv->mInts.mValues;
  (void) rtdvd;
  (void) rtdvi;
  (void) modeVector;
  (void) motionData;
  (void) error;
  return 0;
}

static size_t computeAssemblyError_1(const RuntimeDerivedValuesBundle *rtdv,
  const int *modeVector, const double *motionData, double *error)
{
  const double *rtdvd = rtdv->mDoubles.mValues;
  const int *rtdvi = rtdv->mInts.mValues;
  (void) rtdvd;
  (void) rtdvi;
  (void) modeVector;
  (void) motionData;
  (void) error;
  return 0;
}

static size_t computeAssemblyError_2(const RuntimeDerivedValuesBundle *rtdv,
  const int *modeVector, const double *motionData, double *error)
{
  const double *rtdvd = rtdv->mDoubles.mValues;
  const int *rtdvi = rtdv->mInts.mValues;
  (void) rtdvd;
  (void) rtdvi;
  (void) modeVector;
  (void) motionData;
  (void) error;
  return 0;
}

static size_t computeAssemblyError_3(const RuntimeDerivedValuesBundle *rtdv,
  const int *modeVector, const double *motionData, double *error)
{
  const double *rtdvd = rtdv->mDoubles.mValues;
  const int *rtdvi = rtdv->mInts.mValues;
  double xx[13];
  (void) rtdvd;
  (void) rtdvi;
  (void) modeVector;
  xx[0] = 0.06977021494021335;
  xx[1] = 3.56124026098071e-3;
  xx[2] = xx[0] * motionData[30] + xx[1] * motionData[31];
  xx[3] = motionData[29];
  xx[4] = motionData[30];
  xx[5] = motionData[31];
  xx[6] = xx[0] * motionData[29];
  xx[7] = xx[2];
  xx[8] = - xx[6];
  xx[9] = - (xx[1] * motionData[29]);
  pm_math_Vector3_cross_ra(xx + 3, xx + 7, xx + 10);
  xx[3] = 2.0;
  error[0] = motionData[270] - ((xx[2] * motionData[28] + xx[10]) * xx[3] +
    motionData[32]);
  error[1] = motionData[271] - (xx[3] * (xx[11] - xx[6] * motionData[28]) +
    motionData[33]) + xx[1];
  error[2] = motionData[272] - ((xx[12] - xx[1] * motionData[28] * motionData[29])
    * xx[3] + motionData[34]) - xx[0];
  return 3;
}

static size_t computeAssemblyError_4(const RuntimeDerivedValuesBundle *rtdv,
  const int *modeVector, const double *motionData, double *error)
{
  const double *rtdvd = rtdv->mDoubles.mValues;
  const int *rtdvi = rtdv->mInts.mValues;
  double xx[13];
  (void) rtdvd;
  (void) rtdvi;
  (void) modeVector;
  xx[0] = 0.06977021494021335;
  xx[1] = 3.56124026098071e-3;
  xx[2] = xx[0] * motionData[37] - xx[1] * motionData[38];
  xx[3] = motionData[36];
  xx[4] = motionData[37];
  xx[5] = motionData[38];
  xx[6] = xx[0] * motionData[36];
  xx[7] = xx[2];
  xx[8] = - xx[6];
  xx[9] = xx[1] * motionData[36];
  pm_math_Vector3_cross_ra(xx + 3, xx + 7, xx + 10);
  xx[3] = 2.0;
  error[0] = motionData[284] - ((xx[2] * motionData[35] + xx[10]) * xx[3] +
    motionData[39]);
  error[1] = motionData[285] - (xx[3] * (xx[11] - xx[6] * motionData[35]) +
    motionData[40]) - xx[1];
  error[2] = motionData[286] - ((xx[1] * motionData[35] * motionData[36] + xx[12])
    * xx[3] + motionData[41]) - xx[0];
  return 3;
}

static size_t computeAssemblyError_5(const RuntimeDerivedValuesBundle *rtdv,
  const int *modeVector, const double *motionData, double *error)
{
  const double *rtdvd = rtdv->mDoubles.mValues;
  const int *rtdvi = rtdv->mInts.mValues;
  double xx[10];
  (void) rtdvd;
  (void) rtdvi;
  (void) modeVector;
  xx[0] = motionData[301];
  xx[1] = motionData[302];
  xx[2] = motionData[303];
  xx[3] = motionData[304];
  xx[4] = 0.513;
  xx[5] = 0.27424;
  xx[6] = 8.0e-3;
  pm_math_Quaternion_xform_ra(xx + 0, xx + 4, xx + 7);
  error[0] = xx[7] + motionData[305] + 0.3579665484632664;
  error[1] = xx[8] + motionData[306] + 0.2646;
  error[2] = xx[9] + motionData[307] + 0.1015240013853662;
  return 3;
}

static size_t computeAssemblyError_6(const RuntimeDerivedValuesBundle *rtdv,
  const int *modeVector, const double *motionData, double *error)
{
  const double *rtdvd = rtdv->mDoubles.mValues;
  const int *rtdvi = rtdv->mInts.mValues;
  double xx[10];
  (void) rtdvd;
  (void) rtdvi;
  (void) modeVector;
  xx[0] = motionData[301];
  xx[1] = motionData[302];
  xx[2] = motionData[303];
  xx[3] = motionData[304];
  xx[4] = - 8.94e-3;
  xx[5] = 0.3143400000000001;
  xx[6] = 9.000000000000001e-3;
  pm_math_Quaternion_xform_ra(xx + 0, xx + 4, xx + 7);
  error[0] = xx[7] + motionData[305] + 0.8795972214159852;
  error[1] = xx[8] + motionData[306] + 0.2245;
  error[2] = xx[9] + motionData[307] + 0.1195185737585507;
  return 3;
}

static size_t computeAssemblyError_7(const RuntimeDerivedValuesBundle *rtdv,
  const int *modeVector, const double *motionData, double *error)
{
  const double *rtdvd = rtdv->mDoubles.mValues;
  const int *rtdvi = rtdv->mInts.mValues;
  double xx[10];
  (void) rtdvd;
  (void) rtdvi;
  (void) modeVector;
  xx[0] = motionData[42];
  xx[1] = motionData[43];
  xx[2] = motionData[44];
  xx[3] = motionData[45];
  xx[4] = - 8.94e-3;
  xx[5] = - 0.3143400000000001;
  xx[6] = 9.000000000000001e-3;
  pm_math_Quaternion_xform_ra(xx + 0, xx + 4, xx + 7);
  error[0] = xx[7] + motionData[46] + 0.8795972214159852;
  error[1] = xx[8] + motionData[47] - 0.2245;
  error[2] = xx[9] + motionData[48] + 0.1195185737585507;
  return 3;
}

static size_t computeAssemblyError_8(const RuntimeDerivedValuesBundle *rtdv,
  const int *modeVector, const double *motionData, double *error)
{
  const double *rtdvd = rtdv->mDoubles.mValues;
  const int *rtdvi = rtdv->mInts.mValues;
  double xx[10];
  (void) rtdvd;
  (void) rtdvi;
  (void) modeVector;
  xx[0] = motionData[273];
  xx[1] = motionData[274];
  xx[2] = motionData[275];
  xx[3] = motionData[276];
  xx[4] = 0.5109;
  xx[5] = - 0.284;
  xx[6] = - 0.01433000000000001;
  pm_math_Quaternion_xform_ra(xx + 0, xx + 4, xx + 7);
  error[0] = xx[7] + motionData[277] + 0.3651962861772049;
  error[1] = xx[8] + motionData[278] - 0.2296;
  error[2] = xx[9] + motionData[279] - 0.03930618320795053;
  return 3;
}

static size_t computeAssemblyError_9(const RuntimeDerivedValuesBundle *rtdv,
  const int *modeVector, const double *motionData, double *error)
{
  const double *rtdvd = rtdv->mDoubles.mValues;
  const int *rtdvi = rtdv->mInts.mValues;
  double xx[10];
  (void) rtdvd;
  (void) rtdvi;
  (void) modeVector;
  xx[0] = motionData[273];
  xx[1] = motionData[274];
  xx[2] = motionData[275];
  xx[3] = motionData[276];
  xx[4] = - 8.82e-3;
  xx[5] = - 0.2541;
  xx[6] = - 0.01433000000000001;
  pm_math_Quaternion_xform_ra(xx + 0, xx + 4, xx + 7);
  error[0] = xx[7] + motionData[277] + 0.8845720386103748;
  error[1] = xx[8] + motionData[278] - 0.2595;
  error[2] = xx[9] + motionData[279] - 0.0203930611896637;
  return 3;
}

static size_t computeAssemblyError_10(const RuntimeDerivedValuesBundle *rtdv,
  const int *modeVector, const double *motionData, double *error)
{
  const double *rtdvd = rtdv->mDoubles.mValues;
  const int *rtdvi = rtdv->mInts.mValues;
  double xx[10];
  (void) rtdvd;
  (void) rtdvi;
  (void) modeVector;
  xx[0] = motionData[70];
  xx[1] = motionData[71];
  xx[2] = motionData[72];
  xx[3] = motionData[73];
  xx[4] = - 8.82e-3;
  xx[5] = 0.2541;
  xx[6] = - 0.01433000000000001;
  pm_math_Quaternion_xform_ra(xx + 0, xx + 4, xx + 7);
  error[0] = xx[7] + motionData[74] + 0.8845720386103748;
  error[1] = xx[8] + motionData[75] + 0.2595;
  error[2] = xx[9] + motionData[76] - 0.0203930611896637;
  return 3;
}

static size_t computeAssemblyError_11(const RuntimeDerivedValuesBundle *rtdv,
  const int *modeVector, const double *motionData, double *error)
{
  const double *rtdvd = rtdv->mDoubles.mValues;
  const int *rtdvi = rtdv->mInts.mValues;
  double xx[10];
  (void) rtdvd;
  (void) rtdvi;
  (void) modeVector;
  xx[0] = motionData[322];
  xx[1] = motionData[323];
  xx[2] = motionData[324];
  xx[3] = motionData[325];
  xx[4] = 0.1378799999999999;
  xx[5] = 0.29899;
  xx[6] = - 6.500000000000001e-3;
  pm_math_Quaternion_xform_ra(xx + 0, xx + 4, xx + 7);
  error[0] = xx[7] + motionData[326] - 0.8225246892800159;
  error[1] = xx[8] + motionData[327] + 0.264;
  error[2] = xx[9] + motionData[328] + 0.07704855223048318;
  return 3;
}

static size_t computeAssemblyError_12(const RuntimeDerivedValuesBundle *rtdv,
  const int *modeVector, const double *motionData, double *error)
{
  const double *rtdvd = rtdv->mDoubles.mValues;
  const int *rtdvi = rtdv->mInts.mValues;
  double xx[10];
  (void) rtdvd;
  (void) rtdvi;
  (void) modeVector;
  xx[0] = motionData[322];
  xx[1] = motionData[323];
  xx[2] = motionData[324];
  xx[3] = motionData[325];
  xx[4] = - 0.15472;
  xx[5] = 0.29499;
  xx[6] = - 2.200000000000003e-3;
  pm_math_Quaternion_xform_ra(xx + 0, xx + 4, xx + 7);
  error[0] = xx[7] + motionData[326] - 0.5299620178672857;
  error[1] = xx[8] + motionData[327] + 0.268;
  error[2] = xx[9] + motionData[328] + 0.08339940224027326;
  return 3;
}

static size_t computeAssemblyError_13(const RuntimeDerivedValuesBundle *rtdv,
  const int *modeVector, const double *motionData, double *error)
{
  const double *rtdvd = rtdv->mDoubles.mValues;
  const int *rtdvi = rtdv->mInts.mValues;
  double xx[13];
  (void) rtdvd;
  (void) rtdvi;
  (void) modeVector;
  xx[0] = motionData[7];
  xx[1] = motionData[8];
  xx[2] = motionData[9];
  xx[3] = motionData[10];
  xx[4] = 0.1378799999999999;
  xx[5] = - 0.29899;
  xx[6] = - 6.500000000000001e-3;
  pm_math_Quaternion_xform_ra(xx + 0, xx + 4, xx + 7);
  xx[0] = motionData[336];
  xx[1] = motionData[337];
  xx[2] = motionData[338];
  xx[3] = motionData[339];
  xx[4] = 0.8225246892800159;
  xx[5] = 0.264;
  xx[6] = - 0.07704855223048318;
  pm_math_Quaternion_xform_ra(xx + 0, xx + 4, xx + 10);
  error[0] = xx[7] + motionData[11] - (xx[10] + motionData[340]);
  error[1] = xx[8] + motionData[12] - (xx[11] + motionData[341]);
  error[2] = xx[9] + motionData[13] - (xx[12] + motionData[342]);
  return 3;
}

static size_t computeAssemblyError_14(const RuntimeDerivedValuesBundle *rtdv,
  const int *modeVector, const double *motionData, double *error)
{
  const double *rtdvd = rtdv->mDoubles.mValues;
  const int *rtdvi = rtdv->mInts.mValues;
  double xx[13];
  (void) rtdvd;
  (void) rtdvi;
  (void) modeVector;
  xx[0] = motionData[7];
  xx[1] = motionData[8];
  xx[2] = motionData[9];
  xx[3] = motionData[10];
  xx[4] = - 0.15472;
  xx[5] = - 0.29499;
  xx[6] = - 2.200000000000003e-3;
  pm_math_Quaternion_xform_ra(xx + 0, xx + 4, xx + 7);
  xx[0] = motionData[336];
  xx[1] = motionData[337];
  xx[2] = motionData[338];
  xx[3] = motionData[339];
  xx[4] = 0.5299620178672857;
  xx[5] = 0.268;
  xx[6] = - 0.08339940224027326;
  pm_math_Quaternion_xform_ra(xx + 0, xx + 4, xx + 10);
  error[0] = xx[7] + motionData[11] - (xx[10] + motionData[340]);
  error[1] = xx[8] + motionData[12] - (xx[11] + motionData[341]);
  error[2] = xx[9] + motionData[13] - (xx[12] + motionData[342]);
  return 3;
}

static size_t computeAssemblyError_15(const RuntimeDerivedValuesBundle *rtdv,
  const int *modeVector, const double *motionData, double *error)
{
  const double *rtdvd = rtdv->mDoubles.mValues;
  const int *rtdvi = rtdv->mInts.mValues;
  double xx[10];
  (void) rtdvd;
  (void) rtdvi;
  (void) modeVector;
  xx[0] = motionData[112];
  xx[1] = motionData[113];
  xx[2] = motionData[114];
  xx[3] = motionData[115];
  xx[4] = - 0.14521;
  xx[5] = 0.26683;
  xx[6] = - 0.04444999999999999;
  pm_math_Quaternion_xform_ra(xx + 0, xx + 4, xx + 7);
  error[0] = xx[7] + motionData[116] - 0.5235527553923325;
  error[1] = xx[8] + motionData[117] + 0.2717;
  error[2] = xx[9] + motionData[118] - 0.03764753640142538;
  return 3;
}

static size_t computeAssemblyError_16(const RuntimeDerivedValuesBundle *rtdv,
  const int *modeVector, const double *motionData, double *error)
{
  const double *rtdvd = rtdv->mDoubles.mValues;
  const int *rtdvi = rtdv->mInts.mValues;
  double xx[10];
  (void) rtdvd;
  (void) rtdvi;
  (void) modeVector;
  xx[0] = motionData[21];
  xx[1] = motionData[22];
  xx[2] = motionData[23];
  xx[3] = motionData[24];
  xx[4] = 0.5235527553923325;
  xx[5] = 0.2717;
  xx[6] = 0.03764753640142538;
  pm_math_Quaternion_xform_ra(xx + 0, xx + 4, xx + 7);
  error[0] = - (0.14521 + xx[7] + motionData[25]);
  error[1] = - (0.26683 + xx[8] + motionData[26]);
  error[2] = - (0.04444999999999999 + xx[9] + motionData[27]);
  return 3;
}

static size_t computeAssemblyError_17(const RuntimeDerivedValuesBundle *rtdv,
  const int *modeVector, const double *motionData, double *error)
{
  const double *rtdvd = rtdv->mDoubles.mValues;
  const int *rtdvi = rtdv->mInts.mValues;
  double xx[13];
  (void) rtdvd;
  (void) rtdvi;
  (void) modeVector;
  xx[0] = 3.749811151839773e-3;
  xx[1] = 0.05742288587815435;
  xx[2] = xx[0] * motionData[345] - xx[1] * motionData[346];
  xx[3] = motionData[344];
  xx[4] = motionData[345];
  xx[5] = motionData[346];
  xx[6] = xx[0] * motionData[344];
  xx[7] = xx[2];
  xx[8] = - xx[6];
  xx[9] = xx[1] * motionData[344];
  pm_math_Vector3_cross_ra(xx + 3, xx + 7, xx + 10);
  xx[3] = 2.0;
  error[0] = motionData[144] - ((xx[2] * motionData[343] + xx[10]) * xx[3] +
    motionData[347]);
  error[1] = motionData[145] - (xx[3] * (xx[11] - xx[6] * motionData[343]) +
    motionData[348]) - xx[1];
  error[2] = motionData[146] - ((xx[1] * motionData[343] * motionData[344] + xx
    [12]) * xx[3] + motionData[349]) - xx[0];
  return 3;
}

static size_t computeAssemblyError_18(const RuntimeDerivedValuesBundle *rtdv,
  const int *modeVector, const double *motionData, double *error)
{
  const double *rtdvd = rtdv->mDoubles.mValues;
  const int *rtdvi = rtdv->mInts.mValues;
  double xx[13];
  (void) rtdvd;
  (void) rtdvi;
  (void) modeVector;
  xx[0] = 3.749811151839773e-3;
  xx[1] = 0.05742288587815435;
  xx[2] = xx[0] * motionData[107] + xx[1] * motionData[108];
  xx[3] = motionData[106];
  xx[4] = motionData[107];
  xx[5] = motionData[108];
  xx[6] = xx[0] * motionData[106];
  xx[7] = xx[2];
  xx[8] = - xx[6];
  xx[9] = - (xx[1] * motionData[106]);
  pm_math_Vector3_cross_ra(xx + 3, xx + 7, xx + 10);
  xx[3] = 2.0;
  error[0] = motionData[354] - ((xx[2] * motionData[105] + xx[10]) * xx[3] +
    motionData[109]);
  error[1] = motionData[355] - (xx[3] * (xx[11] - xx[6] * motionData[105]) +
    motionData[110]) + xx[1];
  error[2] = motionData[356] - ((xx[12] - xx[1] * motionData[105] * motionData
    [106]) * xx[3] + motionData[111]) - xx[0];
  return 3;
}

size_t dempsystest_bjorn_59d2bdba_5_computeAssemblyError(const void *mech, const
  RuntimeDerivedValuesBundle *rtdv, size_t constraintIdx, const int *modeVector,
  const double *motionData, double *error)
{
  (void) mech;
  (void)rtdv;
  (void) modeVector;
  (void) motionData;
  (void) error;
  switch (constraintIdx)
  {
   case 0:
    return computeAssemblyError_0(rtdv, modeVector, motionData, error);

   case 1:
    return computeAssemblyError_1(rtdv, modeVector, motionData, error);

   case 2:
    return computeAssemblyError_2(rtdv, modeVector, motionData, error);

   case 3:
    return computeAssemblyError_3(rtdv, modeVector, motionData, error);

   case 4:
    return computeAssemblyError_4(rtdv, modeVector, motionData, error);

   case 5:
    return computeAssemblyError_5(rtdv, modeVector, motionData, error);

   case 6:
    return computeAssemblyError_6(rtdv, modeVector, motionData, error);

   case 7:
    return computeAssemblyError_7(rtdv, modeVector, motionData, error);

   case 8:
    return computeAssemblyError_8(rtdv, modeVector, motionData, error);

   case 9:
    return computeAssemblyError_9(rtdv, modeVector, motionData, error);

   case 10:
    return computeAssemblyError_10(rtdv, modeVector, motionData, error);

   case 11:
    return computeAssemblyError_11(rtdv, modeVector, motionData, error);

   case 12:
    return computeAssemblyError_12(rtdv, modeVector, motionData, error);

   case 13:
    return computeAssemblyError_13(rtdv, modeVector, motionData, error);

   case 14:
    return computeAssemblyError_14(rtdv, modeVector, motionData, error);

   case 15:
    return computeAssemblyError_15(rtdv, modeVector, motionData, error);

   case 16:
    return computeAssemblyError_16(rtdv, modeVector, motionData, error);

   case 17:
    return computeAssemblyError_17(rtdv, modeVector, motionData, error);

   case 18:
    return computeAssemblyError_18(rtdv, modeVector, motionData, error);
  }

  return 0;
}

static size_t computeAssemblyJacobian_0(const RuntimeDerivedValuesBundle *rtdv,
  const double *state, const int *modeVector, const double *motionData, double
  *J)
{
  const double *rtdvd = rtdv->mDoubles.mValues;
  const int *rtdvi = rtdv->mInts.mValues;
  (void) rtdvd;
  (void) rtdvi;
  (void) state;
  (void) modeVector;
  (void) motionData;
  (void) J;
  return 0;
}

static size_t computeAssemblyJacobian_1(const RuntimeDerivedValuesBundle *rtdv,
  const double *state, const int *modeVector, const double *motionData, double
  *J)
{
  const double *rtdvd = rtdv->mDoubles.mValues;
  const int *rtdvi = rtdv->mInts.mValues;
  (void) rtdvd;
  (void) rtdvi;
  (void) state;
  (void) modeVector;
  (void) motionData;
  (void) J;
  return 0;
}

static size_t computeAssemblyJacobian_2(const RuntimeDerivedValuesBundle *rtdv,
  const double *state, const int *modeVector, const double *motionData, double
  *J)
{
  const double *rtdvd = rtdv->mDoubles.mValues;
  const int *rtdvi = rtdv->mInts.mValues;
  (void) rtdvd;
  (void) rtdvi;
  (void) state;
  (void) modeVector;
  (void) motionData;
  (void) J;
  return 0;
}

static size_t computeAssemblyJacobian_3(const RuntimeDerivedValuesBundle *rtdv,
  const double *state, const int *modeVector, const double *motionData, double
  *J)
{
  const double *rtdvd = rtdv->mDoubles.mValues;
  const int *rtdvi = rtdv->mInts.mValues;
  double xx[80];
  (void) rtdvd;
  (void) rtdvi;
  (void) modeVector;
  xx[0] = 0.0;
  xx[1] = 0.9691346498545245;
  xx[2] = 0.5 * state[34];
  xx[3] = cos(xx[2]);
  xx[4] = 0.2458594005622715;
  xx[5] = sin(xx[2]);
  xx[2] = xx[1] * xx[3] - xx[4] * xx[5];
  xx[6] = 0.06977021494021335;
  xx[7] = 4.475014772496788e-3;
  xx[8] = 0.01763972361731614;
  xx[9] = xx[7] * xx[3] + xx[8] * xx[5];
  xx[10] = xx[7] * xx[5] - xx[8] * xx[3];
  xx[7] = 3.56124026098071e-3;
  xx[8] = xx[6] * xx[9] - xx[10] * xx[7];
  xx[11] = xx[1] * xx[5] + xx[4] * xx[3];
  xx[3] = xx[11];
  xx[4] = xx[10];
  xx[5] = xx[9];
  xx[1] = xx[11] * xx[7];
  xx[7] = xx[11] * xx[6];
  xx[12] = xx[8];
  xx[13] = xx[1];
  xx[14] = - xx[7];
  pm_math_Vector3_cross_ra(xx + 3, xx + 12, xx + 15);
  xx[6] = 2.0;
  xx[12] = 0.02673029948304497;
  xx[13] = 0.03945399617968662;
  xx[14] = xx[10] * xx[12] + xx[13] * xx[9];
  xx[9] = xx[11] * xx[12];
  xx[10] = xx[11] * xx[13];
  xx[11] = xx[14];
  xx[12] = - xx[9];
  xx[13] = - xx[10];
  pm_math_Vector3_cross_ra(xx + 3, xx + 11, xx + 18);
  xx[3] = 0.9998343934839863;
  xx[4] = 0.0181985056119827;
  xx[5] = xx[3] * state[68] - xx[4] * state[66];
  xx[11] = xx[5] * motionData[82];
  xx[12] = xx[3] * state[69] + xx[4] * state[67];
  xx[13] = xx[12] * motionData[83];
  xx[21] = xx[11] + xx[13];
  xx[22] = xx[3] * state[66] + xx[4] * state[68];
  xx[23] = xx[3] * state[67] - xx[4] * state[69];
  xx[24] = xx[23];
  xx[25] = xx[5];
  xx[26] = xx[12];
  xx[3] = xx[23] * motionData[82];
  xx[4] = xx[23] * motionData[83];
  xx[27] = xx[21];
  xx[28] = - xx[3];
  xx[29] = - xx[4];
  pm_math_Vector3_cross_ra(xx + 24, xx + 27, xx + 30);
  xx[27] = 0.01433000000000001;
  xx[28] = xx[27] * xx[12];
  xx[29] = 0.284;
  xx[33] = xx[5] * xx[29];
  xx[34] = xx[28] - xx[33];
  xx[35] = xx[23] * xx[29];
  xx[36] = xx[23] * xx[27];
  xx[37] = xx[34];
  xx[38] = xx[35];
  xx[39] = - xx[36];
  pm_math_Vector3_cross_ra(xx + 24, xx + 37, xx + 40);
  xx[37] = xx[5] * motionData[81];
  xx[38] = xx[23] * motionData[81];
  xx[39] = xx[13] + xx[38];
  xx[13] = xx[5] * motionData[83];
  xx[43] = - xx[37];
  xx[44] = xx[39];
  xx[45] = - xx[13];
  pm_math_Vector3_cross_ra(xx + 24, xx + 43, xx + 46);
  xx[43] = 0.5109;
  xx[44] = xx[5] * xx[43];
  xx[45] = xx[23] * xx[43];
  xx[23] = xx[28] - xx[45];
  xx[28] = xx[5] * xx[27];
  xx[49] = xx[44];
  xx[50] = xx[23];
  xx[51] = - xx[28];
  pm_math_Vector3_cross_ra(xx + 24, xx + 49, xx + 52);
  xx[5] = xx[12] * motionData[81];
  xx[49] = xx[12] * motionData[82];
  xx[50] = xx[38] + xx[11];
  xx[55] = - xx[5];
  xx[56] = - xx[49];
  xx[57] = xx[50];
  pm_math_Vector3_cross_ra(xx + 24, xx + 55, xx + 58);
  xx[11] = xx[43] * xx[12];
  xx[38] = xx[29] * xx[12];
  xx[12] = xx[45] + xx[33];
  xx[55] = xx[11];
  xx[56] = xx[38];
  xx[57] = - xx[12];
  pm_math_Vector3_cross_ra(xx + 24, xx + 55, xx + 61);
  xx[64] = motionData[70];
  xx[65] = motionData[71];
  xx[66] = motionData[72];
  xx[67] = motionData[73];
  xx[24] = 0.1764500000000001;
  xx[25] = xx[24] * state[75];
  xx[26] = 0.20773;
  xx[33] = xx[26] * state[76];
  xx[45] = xx[25] + xx[33];
  xx[55] = state[74];
  xx[56] = state[75];
  xx[57] = state[76];
  xx[51] = xx[24] * state[74];
  xx[68] = xx[45];
  xx[69] = - xx[51];
  xx[70] = - (xx[26] * state[74]);
  pm_math_Vector3_cross_ra(xx + 55, xx + 68, xx + 71);
  xx[68] = (xx[45] * state[73] + xx[71]) * xx[6];
  xx[69] = xx[6] * (xx[72] - xx[51] * state[73]) - xx[26];
  xx[70] = xx[24] + (xx[73] - xx[26] * state[73] * state[74]) * xx[6];
  pm_math_Quaternion_xform_ra(xx + 64, xx + 68, xx + 71);
  xx[45] = 7.5e-3;
  xx[51] = xx[45] * state[74];
  xx[68] = xx[33] + xx[51];
  xx[33] = xx[26] * state[75];
  xx[74] = - (xx[45] * state[75]);
  xx[75] = xx[68];
  xx[76] = - xx[33];
  pm_math_Vector3_cross_ra(xx + 55, xx + 74, xx + 77);
  xx[74] = xx[26] + (xx[77] - xx[45] * state[73] * state[75]) * xx[6];
  xx[75] = (xx[68] * state[73] + xx[78]) * xx[6];
  xx[76] = xx[6] * (xx[79] - xx[33] * state[73]) - xx[45];
  pm_math_Quaternion_xform_ra(xx + 64, xx + 74, xx + 68);
  xx[26] = xx[45] * state[76];
  xx[33] = xx[51] + xx[25];
  xx[74] = - xx[26];
  xx[75] = - (xx[24] * state[76]);
  xx[76] = xx[33];
  pm_math_Vector3_cross_ra(xx + 55, xx + 74, xx + 77);
  xx[55] = xx[6] * (xx[77] - xx[26] * state[73]) - xx[24];
  xx[56] = xx[45] + (xx[78] - xx[24] * state[73] * state[76]) * xx[6];
  xx[57] = (xx[33] * state[73] + xx[79]) * xx[6];
  pm_math_Quaternion_xform_ra(xx + 64, xx + 55, xx + 24);
  J[15] = - ((xx[2] * xx[8] + xx[15]) * xx[6] + (xx[14] * xx[2] + xx[18]) * xx[6]);
  J[29] = (xx[21] * xx[22] + xx[30]) * xx[6] + (xx[22] * xx[34] + xx[40]) * xx[6];
  J[30] = motionData[83] + xx[6] * (xx[46] - xx[37] * xx[22]) + (xx[44] * xx[22]
    + xx[52]) * xx[6] + xx[27];
  J[31] = xx[6] * (xx[58] - xx[5] * xx[22]) + xx[6] * (xx[61] + xx[11] * xx[22])
    - motionData[82] + xx[29];
  J[32] = xx[71];
  J[33] = xx[68];
  J[34] = xx[24];
  J[73] = - (xx[6] * (xx[16] + xx[1] * xx[2]) + xx[6] * (xx[19] - xx[9] * xx[2])
             - 0.1092242111199);
  J[87] = xx[6] * (xx[31] - xx[3] * xx[22]) + xx[6] * (xx[41] + xx[35] * xx[22])
    - motionData[83] - xx[27];
  J[88] = (xx[39] * xx[22] + xx[47]) * xx[6] + (xx[23] * xx[22] + xx[53]) * xx[6];
  J[89] = motionData[81] + xx[6] * (xx[59] - xx[49] * xx[22]) + (xx[38] * xx[22]
    + xx[62]) * xx[6] - xx[43];
  J[90] = xx[72];
  J[91] = xx[69];
  J[92] = xx[25];
  J[131] = - ((xx[17] - xx[7] * xx[2]) * xx[6] + (xx[20] - xx[10] * xx[2]) * xx
              [6] + 0.02316905922206426);
  J[145] = motionData[82] + xx[6] * (xx[32] - xx[4] * xx[22]) + (xx[42] - xx[36]
    * xx[22]) * xx[6] - xx[29];
  J[146] = xx[6] * (xx[48] - xx[13] * xx[22]) + xx[6] * (xx[54] - xx[28] * xx[22])
    - motionData[81] + xx[43];
  J[147] = (xx[50] * xx[22] + xx[60]) * xx[6] + (xx[63] - xx[22] * xx[12]) * xx
    [6];
  J[148] = xx[73];
  J[149] = xx[70];
  J[150] = xx[26];
  return 3;
}

static size_t computeAssemblyJacobian_4(const RuntimeDerivedValuesBundle *rtdv,
  const double *state, const int *modeVector, const double *motionData, double
  *J)
{
  const double *rtdvd = rtdv->mDoubles.mValues;
  const int *rtdvi = rtdv->mInts.mValues;
  double xx[101];
  (void) rtdvd;
  (void) rtdvi;
  (void) modeVector;
  xx[0] = 0.0;
  xx[1] = 0.06977021494021335;
  xx[2] = 0.01763972361731614;
  xx[3] = 0.5 * state[36];
  xx[4] = sin(xx[3]);
  xx[5] = 4.475014772496788e-3;
  xx[6] = cos(xx[3]);
  xx[3] = xx[2] * xx[4] - xx[5] * xx[6];
  xx[7] = xx[2] * xx[6] + xx[5] * xx[4];
  xx[2] = 3.56124026098071e-3;
  xx[5] = xx[1] * xx[3] - xx[7] * xx[2];
  xx[8] = 0.9691346498545245;
  xx[9] = 0.2458594005622715;
  xx[10] = xx[8] * xx[6] + xx[9] * xx[4];
  xx[11] = xx[8] * xx[4] - xx[9] * xx[6];
  xx[12] = xx[11];
  xx[13] = - xx[7];
  xx[14] = xx[3];
  xx[4] = xx[11] * xx[2];
  xx[2] = xx[11] * xx[1];
  xx[15] = xx[5];
  xx[16] = - xx[4];
  xx[17] = - xx[2];
  pm_math_Vector3_cross_ra(xx + 12, xx + 15, xx + 18);
  xx[1] = 2.0;
  xx[6] = 0.02673029948304497;
  xx[8] = 0.03945399617968662;
  xx[9] = xx[7] * xx[6] + xx[8] * xx[3];
  xx[3] = xx[11] * xx[6];
  xx[6] = xx[11] * xx[8];
  xx[15] = xx[9];
  xx[16] = xx[3];
  xx[17] = - xx[6];
  pm_math_Vector3_cross_ra(xx + 12, xx + 15, xx + 21);
  xx[7] = 0.9998343934839863;
  xx[8] = 0.0181985056119827;
  xx[11] = xx[7] * state[40] - xx[8] * state[38];
  xx[12] = xx[11] * motionData[299];
  xx[13] = xx[7] * state[41] + xx[8] * state[39];
  xx[14] = xx[13] * motionData[300];
  xx[15] = xx[12] + xx[14];
  xx[16] = xx[7] * state[38] + xx[8] * state[40];
  xx[17] = xx[7] * state[39] - xx[8] * state[41];
  xx[24] = xx[17];
  xx[25] = xx[11];
  xx[26] = xx[13];
  xx[7] = xx[17] * motionData[299];
  xx[8] = xx[17] * motionData[300];
  xx[27] = xx[15];
  xx[28] = - xx[7];
  xx[29] = - xx[8];
  pm_math_Vector3_cross_ra(xx + 24, xx + 27, xx + 30);
  xx[27] = 0.27424;
  xx[28] = xx[11] * xx[27];
  xx[29] = 8.0e-3;
  xx[33] = xx[29] * xx[13];
  xx[34] = xx[28] - xx[33];
  xx[35] = xx[17] * xx[27];
  xx[36] = xx[17] * xx[29];
  xx[37] = xx[34];
  xx[38] = - xx[35];
  xx[39] = xx[36];
  pm_math_Vector3_cross_ra(xx + 24, xx + 37, xx + 40);
  xx[37] = xx[11] * motionData[298];
  xx[38] = xx[17] * motionData[298];
  xx[39] = xx[14] + xx[38];
  xx[14] = xx[11] * motionData[300];
  xx[43] = - xx[37];
  xx[44] = xx[39];
  xx[45] = - xx[14];
  pm_math_Vector3_cross_ra(xx + 24, xx + 43, xx + 46);
  xx[43] = 0.513;
  xx[44] = xx[11] * xx[43];
  xx[45] = xx[17] * xx[43];
  xx[17] = xx[33] + xx[45];
  xx[33] = xx[11] * xx[29];
  xx[49] = xx[44];
  xx[50] = - xx[17];
  xx[51] = xx[33];
  pm_math_Vector3_cross_ra(xx + 24, xx + 49, xx + 52);
  xx[11] = xx[13] * motionData[298];
  xx[49] = xx[13] * motionData[299];
  xx[50] = xx[38] + xx[12];
  xx[55] = - xx[11];
  xx[56] = - xx[49];
  xx[57] = xx[50];
  pm_math_Vector3_cross_ra(xx + 24, xx + 55, xx + 58);
  xx[12] = xx[43] * xx[13];
  xx[38] = xx[27] * xx[13];
  xx[13] = xx[28] - xx[45];
  xx[55] = xx[12];
  xx[56] = - xx[38];
  xx[57] = xx[13];
  pm_math_Vector3_cross_ra(xx + 24, xx + 55, xx + 61);
  xx[64] = motionData[42];
  xx[65] = motionData[43];
  xx[66] = motionData[44];
  xx[67] = motionData[45];
  xx[24] = 0.9999999561635808;
  xx[25] = 2.960959925766019e-4;
  xx[26] = xx[24] * state[45] - xx[25] * state[46];
  xx[28] = xx[25] * state[45] + xx[24] * state[46];
  xx[45] = xx[24] * state[47] - xx[25] * state[48];
  xx[51] = xx[24] * state[48] + xx[25] * state[47];
  xx[68] = - xx[26];
  xx[69] = xx[28];
  xx[70] = xx[45];
  xx[71] = xx[51];
  xx[55] = xx[45] * xx[45];
  xx[56] = xx[51] * xx[51];
  xx[57] = 1.0;
  xx[72] = (xx[55] + xx[56]) * xx[1] - xx[57];
  xx[73] = xx[26] * xx[51];
  xx[74] = xx[28] * xx[45];
  xx[75] = xx[73] + xx[74];
  xx[76] = xx[26] * xx[45];
  xx[77] = xx[28] * xx[51];
  xx[78] = xx[76] - xx[77];
  xx[79] = xx[72];
  xx[80] = - (xx[75] * xx[1]);
  xx[81] = xx[1] * xx[78];
  xx[82] = motionData[291];
  xx[83] = motionData[292];
  xx[84] = motionData[293];
  pm_math_Vector3_cross_ra(xx + 79, xx + 82, xx + 85);
  pm_math_Quaternion_xform_ra(xx + 68, xx + 85, xx + 79);
  xx[85] = 0.283893415005525;
  xx[86] = 0.1300676268524588;
  xx[87] = 0.1419467075027625;
  xx[88] = 0.06503381342622938;
  xx[89] = - (xx[75] * xx[85] + xx[86] * xx[78]);
  xx[90] = - (xx[72] * xx[87]);
  xx[91] = xx[72] * xx[88];
  pm_math_Quaternion_xform_ra(xx + 68, xx + 89, xx + 92);
  xx[89] = xx[79] + xx[92];
  xx[90] = xx[80] + xx[93];
  xx[91] = xx[81] + xx[94];
  pm_math_Quaternion_xform_ra(xx + 64, xx + 89, xx + 78);
  xx[72] = xx[1] * (xx[73] - xx[74]);
  xx[73] = xx[28] * xx[28];
  xx[74] = (xx[56] + xx[73]) * xx[1] - xx[57];
  xx[56] = xx[26] * xx[28];
  xx[26] = xx[51] * xx[45];
  xx[28] = xx[56] + xx[26];
  xx[89] = xx[72];
  xx[90] = xx[74];
  xx[91] = - (xx[28] * xx[1]);
  pm_math_Vector3_cross_ra(xx + 89, xx + 82, xx + 92);
  pm_math_Quaternion_xform_ra(xx + 68, xx + 92, xx + 89);
  xx[92] = xx[74] * xx[87] + xx[28] * xx[86];
  xx[93] = - (xx[87] * xx[72]);
  xx[94] = xx[88] * xx[72];
  pm_math_Quaternion_xform_ra(xx + 68, xx + 92, xx + 95);
  xx[92] = xx[89] + xx[95];
  xx[93] = xx[90] + xx[96];
  xx[94] = xx[91] + xx[97];
  pm_math_Quaternion_xform_ra(xx + 64, xx + 92, xx + 89);
  xx[28] = (xx[76] + xx[77]) * xx[1];
  xx[45] = xx[56] - xx[26];
  xx[26] = (xx[73] + xx[55]) * xx[1] - xx[57];
  xx[72] = - xx[28];
  xx[73] = xx[1] * xx[45];
  xx[74] = xx[26];
  pm_math_Vector3_cross_ra(xx + 72, xx + 82, xx + 75);
  pm_math_Quaternion_xform_ra(xx + 68, xx + 75, xx + 72);
  xx[75] = xx[85] * xx[45] - xx[26] * xx[88];
  xx[76] = xx[87] * xx[28];
  xx[77] = - (xx[88] * xx[28]);
  pm_math_Quaternion_xform_ra(xx + 68, xx + 75, xx + 81);
  xx[68] = xx[72] + xx[81];
  xx[69] = xx[73] + xx[82];
  xx[70] = xx[74] + xx[83];
  pm_math_Quaternion_xform_ra(xx + 64, xx + 68, xx + 71);
  xx[64] = motionData[210];
  xx[65] = motionData[211];
  xx[66] = motionData[212];
  xx[67] = motionData[213];
  xx[26] = xx[24] * state[52] + xx[25] * state[53];
  xx[28] = xx[25] * state[52] - xx[24] * state[53];
  xx[45] = xx[25] * state[55] - xx[24] * state[54];
  xx[51] = xx[24] * state[55] + xx[25] * state[54];
  xx[24] = - xx[51];
  xx[74] = xx[26];
  xx[75] = xx[28];
  xx[76] = xx[45];
  xx[77] = xx[24];
  xx[68] = (xx[45] * xx[45] + xx[51] * xx[51]) * xx[1] - xx[57];
  xx[69] = - ((xx[26] * xx[51] + xx[28] * xx[45]) * xx[1]);
  xx[70] = xx[1] * (xx[51] * xx[28] - xx[26] * xx[45]);
  xx[55] = motionData[67];
  xx[56] = motionData[68];
  xx[57] = motionData[69];
  pm_math_Vector3_cross_ra(xx + 68, xx + 55, xx + 81);
  pm_math_Quaternion_xform_ra(xx + 74, xx + 81, xx + 68);
  pm_math_Quaternion_xform_ra(xx + 64, xx + 68, xx + 81);
  xx[68] = xx[28];
  xx[69] = xx[45];
  xx[70] = xx[24];
  xx[24] = 5.921919591936278e-4;
  xx[25] = 0.9999998246543264;
  xx[84] = xx[24] * xx[45] + xx[51] * xx[25];
  xx[85] = xx[24] * xx[28];
  xx[86] = xx[25] * xx[28];
  xx[92] = - xx[84];
  xx[93] = xx[85];
  xx[94] = - xx[86];
  pm_math_Vector3_cross_ra(xx + 68, xx + 92, xx + 95);
  xx[28] = xx[26] * xx[85];
  xx[87] = xx[26] * xx[86];
  xx[92] = xx[1] * (xx[95] + xx[26] * xx[84]);
  xx[93] = (xx[96] - xx[28]) * xx[1] - xx[25];
  xx[94] = xx[1] * (xx[97] + xx[87]) - xx[24];
  pm_math_Vector3_cross_ra(xx + 92, xx + 55, xx + 95);
  pm_math_Quaternion_xform_ra(xx + 74, xx + 95, xx + 92);
  pm_math_Quaternion_xform_ra(xx + 64, xx + 92, xx + 95);
  xx[84] = xx[51] * xx[24] - xx[25] * xx[45];
  xx[92] = xx[84];
  xx[93] = xx[86];
  xx[94] = xx[85];
  pm_math_Vector3_cross_ra(xx + 68, xx + 92, xx + 98);
  xx[68] = xx[1] * (xx[98] - xx[26] * xx[84]);
  xx[69] = xx[24] + (xx[99] - xx[87]) * xx[1];
  xx[70] = xx[1] * (xx[100] - xx[28]) - xx[25];
  pm_math_Vector3_cross_ra(xx + 68, xx + 55, xx + 24);
  pm_math_Quaternion_xform_ra(xx + 74, xx + 24, xx + 55);
  pm_math_Quaternion_xform_ra(xx + 64, xx + 55, xx + 24);
  xx[64] = motionData[273];
  xx[65] = motionData[274];
  xx[66] = motionData[275];
  xx[67] = motionData[276];
  xx[28] = 0.20773;
  xx[45] = xx[28] * state[62];
  xx[51] = 0.1764500000000001;
  xx[55] = xx[51] * state[61];
  xx[56] = xx[45] - xx[55];
  xx[68] = state[60];
  xx[69] = state[61];
  xx[70] = state[62];
  xx[57] = xx[51] * state[60];
  xx[74] = xx[56];
  xx[75] = xx[57];
  xx[76] = - (xx[28] * state[60]);
  pm_math_Vector3_cross_ra(xx + 68, xx + 74, xx + 84);
  xx[74] = (xx[56] * state[59] + xx[84]) * xx[1];
  xx[75] = xx[1] * (xx[85] + xx[57] * state[59]) - xx[28];
  xx[76] = (xx[86] - xx[28] * state[59] * state[60]) * xx[1] - xx[51];
  pm_math_Quaternion_xform_ra(xx + 64, xx + 74, xx + 84);
  xx[56] = 7.5e-3;
  xx[57] = xx[56] * state[60];
  xx[74] = xx[45] + xx[57];
  xx[45] = xx[28] * state[61];
  xx[75] = - (xx[56] * state[61]);
  xx[76] = xx[74];
  xx[77] = - xx[45];
  pm_math_Vector3_cross_ra(xx + 68, xx + 75, xx + 92);
  xx[75] = xx[28] + (xx[92] - xx[56] * state[59] * state[61]) * xx[1];
  xx[76] = (xx[74] * state[59] + xx[93]) * xx[1];
  xx[77] = xx[1] * (xx[94] - xx[45] * state[59]) - xx[56];
  pm_math_Quaternion_xform_ra(xx + 64, xx + 75, xx + 92);
  xx[28] = xx[56] * state[62];
  xx[45] = xx[57] - xx[55];
  xx[74] = - xx[28];
  xx[75] = xx[51] * state[62];
  xx[76] = xx[45];
  pm_math_Vector3_cross_ra(xx + 68, xx + 74, xx + 98);
  xx[68] = xx[51] + xx[1] * (xx[98] - xx[28] * state[59]);
  xx[69] = xx[56] + (xx[51] * state[59] * state[62] + xx[99]) * xx[1];
  xx[70] = (xx[45] * state[59] + xx[100]) * xx[1];
  pm_math_Quaternion_xform_ra(xx + 64, xx + 68, xx + 55);
  J[16] = - ((xx[5] * xx[10] + xx[18]) * xx[1] + (xx[10] * xx[9] + xx[21]) * xx
             [1]);
  J[17] = (xx[15] * xx[16] + xx[30]) * xx[1] + (xx[34] * xx[16] + xx[40]) * xx[1];
  J[18] = motionData[300] + xx[1] * (xx[46] - xx[37] * xx[16]) + (xx[44] * xx[16]
    + xx[52]) * xx[1] - xx[29];
  J[19] = xx[1] * (xx[58] - xx[11] * xx[16]) + xx[1] * (xx[61] + xx[12] * xx[16])
    - motionData[299] - xx[27];
  J[20] = xx[78];
  J[21] = xx[89];
  J[22] = xx[71];
  J[23] = xx[81];
  J[24] = xx[95];
  J[25] = xx[24];
  J[26] = xx[84];
  J[27] = xx[92];
  J[28] = xx[55];
  J[74] = - (xx[1] * (xx[19] - xx[4] * xx[10]) + xx[1] * (xx[22] + xx[3] * xx[10])
             - 0.1092242111199);
  J[75] = xx[1] * (xx[31] - xx[7] * xx[16]) + xx[1] * (xx[41] - xx[35] * xx[16])
    - motionData[300] + xx[29];
  J[76] = (xx[39] * xx[16] + xx[47]) * xx[1] + (xx[53] - xx[16] * xx[17]) * xx[1];
  J[77] = motionData[298] + xx[1] * (xx[59] - xx[49] * xx[16]) + (xx[62] - xx[38]
    * xx[16]) * xx[1] - xx[43];
  J[78] = xx[79];
  J[79] = xx[90];
  J[80] = xx[72];
  J[81] = xx[82];
  J[82] = xx[96];
  J[83] = xx[25];
  J[84] = xx[85];
  J[85] = xx[93];
  J[86] = xx[56];
  J[132] = - ((xx[20] - xx[2] * xx[10]) * xx[1] + (xx[23] - xx[6] * xx[10]) *
              xx[1] - 0.02316905922206426);
  J[133] = motionData[299] + xx[1] * (xx[32] - xx[8] * xx[16]) + (xx[36] * xx[16]
    + xx[42]) * xx[1] + xx[27];
  J[134] = xx[1] * (xx[48] - xx[14] * xx[16]) + xx[1] * (xx[54] + xx[33] * xx[16])
    - motionData[298] + xx[43];
  J[135] = (xx[50] * xx[16] + xx[60]) * xx[1] + (xx[16] * xx[13] + xx[63]) * xx
    [1];
  J[136] = xx[80];
  J[137] = xx[91];
  J[138] = xx[73];
  J[139] = xx[83];
  J[140] = xx[97];
  J[141] = xx[26];
  J[142] = xx[86];
  J[143] = xx[94];
  J[144] = xx[57];
  return 3;
}

static size_t computeAssemblyJacobian_5(const RuntimeDerivedValuesBundle *rtdv,
  const double *state, const int *modeVector, const double *motionData, double
  *J)
{
  const double *rtdvd = rtdv->mDoubles.mValues;
  const int *rtdvi = rtdv->mInts.mValues;
  double xx[102];
  (void) rtdvd;
  (void) rtdvi;
  (void) modeVector;
  xx[0] = 0.0;
  xx[1] = 0.9998343934839863;
  xx[2] = 0.0181985056119827;
  xx[3] = xx[1] * state[66] + xx[2] * state[68];
  xx[4] = xx[1] * state[67] - xx[2] * state[69];
  xx[5] = xx[1] * state[68] - xx[2] * state[66];
  xx[6] = xx[1] * state[69] + xx[2] * state[67];
  xx[7] = motionData[308];
  xx[8] = motionData[309];
  xx[9] = motionData[310];
  xx[10] = motionData[311];
  pm_math_Quaternion_compose_ra(xx + 3, xx + 7, xx + 11);
  xx[1] = 1.0;
  xx[2] = motionData[310] * motionData[310];
  xx[7] = motionData[311] * motionData[311];
  xx[8] = 2.0;
  xx[9] = motionData[309] * motionData[310];
  xx[10] = motionData[308] * motionData[311];
  xx[15] = motionData[308] * motionData[310];
  xx[16] = motionData[309] * motionData[311];
  xx[17] = xx[1] - (xx[2] + xx[7]) * xx[8];
  xx[18] = xx[8] * (xx[9] - xx[10]);
  xx[19] = (xx[15] + xx[16]) * xx[8];
  xx[20] = 0.513;
  xx[21] = 0.27424;
  xx[22] = 8.0e-3;
  pm_math_Vector3_cross_ra(xx + 17, xx + 20, xx + 23);
  pm_math_Quaternion_xform_ra(xx + 11, xx + 23, xx + 17);
  xx[23] = xx[5] * motionData[313];
  xx[24] = xx[6] * motionData[314];
  xx[25] = xx[23] + xx[24];
  xx[26] = xx[4] * motionData[313];
  xx[27] = xx[4] * motionData[314];
  xx[28] = xx[25];
  xx[29] = - xx[26];
  xx[30] = - xx[27];
  pm_math_Vector3_cross_ra(xx + 4, xx + 28, xx + 31);
  xx[28] = 0.01433000000000001;
  xx[29] = xx[28] * xx[6];
  xx[30] = 0.284;
  xx[34] = xx[5] * xx[30];
  xx[35] = xx[29] - xx[34];
  xx[36] = xx[4] * xx[30];
  xx[37] = xx[4] * xx[28];
  xx[38] = xx[35];
  xx[39] = xx[36];
  xx[40] = - xx[37];
  pm_math_Vector3_cross_ra(xx + 4, xx + 38, xx + 41);
  xx[38] = motionData[309] * motionData[309];
  xx[39] = motionData[310] * motionData[311];
  xx[40] = motionData[308] * motionData[309];
  xx[44] = (xx[10] + xx[9]) * xx[8];
  xx[45] = xx[1] - (xx[7] + xx[38]) * xx[8];
  xx[46] = xx[8] * (xx[39] - xx[40]);
  pm_math_Vector3_cross_ra(xx + 44, xx + 20, xx + 47);
  pm_math_Quaternion_xform_ra(xx + 11, xx + 47, xx + 44);
  xx[7] = xx[5] * motionData[312];
  xx[9] = xx[4] * motionData[312];
  xx[10] = xx[24] + xx[9];
  xx[24] = xx[5] * motionData[314];
  xx[47] = - xx[7];
  xx[48] = xx[10];
  xx[49] = - xx[24];
  pm_math_Vector3_cross_ra(xx + 4, xx + 47, xx + 50);
  xx[47] = 0.5109;
  xx[48] = xx[5] * xx[47];
  xx[49] = xx[4] * xx[47];
  xx[53] = xx[29] - xx[49];
  xx[29] = xx[5] * xx[28];
  xx[54] = xx[48];
  xx[55] = xx[53];
  xx[56] = - xx[29];
  pm_math_Vector3_cross_ra(xx + 4, xx + 54, xx + 57);
  xx[54] = xx[8] * (xx[16] - xx[15]);
  xx[55] = (xx[40] + xx[39]) * xx[8];
  xx[56] = xx[1] - (xx[38] + xx[2]) * xx[8];
  pm_math_Vector3_cross_ra(xx + 54, xx + 20, xx + 38);
  pm_math_Quaternion_xform_ra(xx + 11, xx + 38, xx + 54);
  xx[2] = xx[6] * motionData[312];
  xx[11] = xx[6] * motionData[313];
  xx[12] = xx[9] + xx[23];
  xx[13] = - xx[2];
  xx[14] = - xx[11];
  xx[15] = xx[12];
  pm_math_Vector3_cross_ra(xx + 4, xx + 13, xx + 38);
  xx[9] = xx[47] * xx[6];
  xx[13] = xx[30] * xx[6];
  xx[14] = xx[49] + xx[34];
  xx[60] = xx[9];
  xx[61] = xx[13];
  xx[62] = - xx[14];
  pm_math_Vector3_cross_ra(xx + 4, xx + 60, xx + 63);
  xx[66] = motionData[70];
  xx[67] = motionData[71];
  xx[68] = motionData[72];
  xx[69] = motionData[73];
  xx[4] = 2.960959925763799e-4;
  xx[5] = 0.9999999561635808;
  xx[6] = xx[4] * state[81] - xx[5] * state[80];
  xx[15] = xx[4] * state[80] + xx[5] * state[81];
  xx[16] = - xx[15];
  xx[23] = xx[4] * state[83] + xx[5] * state[82];
  xx[34] = - xx[23];
  xx[49] = xx[5] * state[83] - xx[4] * state[82];
  xx[60] = - xx[49];
  xx[70] = xx[6];
  xx[71] = xx[16];
  xx[72] = xx[34];
  xx[73] = xx[60];
  pm_math_Quaternion_compose_ra(xx + 66, xx + 70, xx + 74);
  xx[78] = motionData[91];
  xx[79] = motionData[92];
  xx[80] = motionData[93];
  xx[81] = motionData[94];
  pm_math_Quaternion_compose_ra(xx + 74, xx + 78, xx + 82);
  xx[74] = xx[1] - (motionData[93] * motionData[93] + motionData[94] *
                    motionData[94]) * xx[8];
  xx[75] = xx[8] * (motionData[92] * motionData[93] - motionData[91] *
                    motionData[94]);
  xx[76] = (motionData[91] * motionData[93] + motionData[92] * motionData[94]) *
    xx[8];
  pm_math_Vector3_cross_ra(xx + 74, xx + 20, xx + 77);
  pm_math_Quaternion_xform_ra(xx + 82, xx + 77, xx + 74);
  xx[77] = xx[16];
  xx[78] = xx[34];
  xx[79] = xx[60];
  xx[1] = xx[23] * motionData[96] + xx[49] * motionData[97];
  xx[16] = xx[15] * motionData[96];
  xx[34] = xx[15] * motionData[97];
  xx[60] = - xx[1];
  xx[61] = xx[16];
  xx[62] = xx[34];
  pm_math_Vector3_cross_ra(xx + 77, xx + 60, xx + 86);
  xx[60] = 0.09037053171319967;
  xx[61] = 0.0213683169329786;
  xx[62] = xx[60] * xx[23] + xx[49] * xx[61];
  xx[80] = xx[60] * xx[15];
  xx[81] = xx[61] * xx[15];
  xx[89] = xx[62];
  xx[90] = - xx[80];
  xx[91] = - xx[81];
  pm_math_Vector3_cross_ra(xx + 77, xx + 89, xx + 92);
  xx[77] = (xx[86] - xx[1] * xx[6]) * xx[8] + (xx[62] * xx[6] + xx[92]) * xx[8];
  xx[78] = xx[8] * (xx[87] + xx[16] * xx[6]) + xx[8] * (xx[93] - xx[80] * xx[6])
    - motionData[97] + xx[61];
  xx[79] = motionData[96] + xx[8] * (xx[88] + xx[34] * xx[6]) + (xx[94] - xx[81]
    * xx[6]) * xx[8] - xx[60];
  pm_math_Quaternion_xform_ra(xx + 66, xx + 77, xx + 60);
  xx[77] = motionData[92];
  xx[78] = motionData[93];
  xx[79] = motionData[94];
  xx[1] = 5.921919591931837e-4;
  xx[16] = 0.9999998246543264;
  xx[34] = xx[1] * motionData[93] + xx[16] * motionData[94];
  xx[80] = xx[1] * motionData[92];
  xx[81] = xx[16] * motionData[92];
  xx[86] = - xx[34];
  xx[87] = xx[80];
  xx[88] = xx[81];
  pm_math_Vector3_cross_ra(xx + 77, xx + 86, xx + 89);
  xx[86] = motionData[91] * motionData[92];
  xx[92] = xx[8] * (xx[89] + xx[34] * motionData[91]);
  xx[93] = xx[16] + (xx[90] - xx[80] * motionData[91]) * xx[8];
  xx[94] = xx[8] * (xx[91] - xx[16] * xx[86]) - xx[1];
  pm_math_Vector3_cross_ra(xx + 92, xx + 20, xx + 87);
  pm_math_Quaternion_xform_ra(xx + 82, xx + 87, xx + 90);
  xx[34] = - (xx[1] * motionData[95]);
  xx[87] = xx[16] * motionData[95];
  xx[93] = xx[16] * motionData[97] + xx[1] * motionData[96];
  xx[94] = xx[34];
  xx[95] = - xx[87];
  pm_math_Quaternion_xform_ra(xx + 70, xx + 93, xx + 96);
  xx[88] = 0.02142182988836524;
  xx[89] = xx[88] * xx[23];
  xx[93] = xx[49] * xx[88];
  xx[99] = xx[96] + (xx[89] * xx[23] + xx[49] * xx[93]) * xx[8] - xx[88];
  xx[100] = xx[97] + xx[8] * (xx[93] * xx[6] - xx[89] * xx[15]);
  xx[101] = xx[98] - (xx[89] * xx[6] + xx[93] * xx[15]) * xx[8];
  pm_math_Quaternion_xform_ra(xx + 66, xx + 99, xx + 93);
  xx[88] = xx[16] * motionData[93] - xx[1] * motionData[94];
  xx[96] = xx[88];
  xx[97] = - xx[81];
  xx[98] = xx[80];
  pm_math_Vector3_cross_ra(xx + 77, xx + 96, xx + 99);
  xx[77] = xx[8] * (xx[99] - xx[88] * motionData[91]);
  xx[78] = xx[1] + (xx[81] * motionData[91] + xx[100]) * xx[8];
  xx[79] = xx[16] + xx[8] * (xx[101] - xx[1] * xx[86]);
  pm_math_Vector3_cross_ra(xx + 77, xx + 20, xx + 96);
  pm_math_Quaternion_xform_ra(xx + 82, xx + 96, xx + 77);
  xx[80] = xx[1] * motionData[97] - xx[16] * motionData[96];
  xx[81] = xx[87];
  xx[82] = xx[34];
  pm_math_Quaternion_xform_ra(xx + 70, xx + 80, xx + 83);
  xx[1] = 0.0903578617216487;
  xx[16] = xx[1] * xx[23];
  xx[34] = xx[49] * xx[1];
  xx[70] = xx[83] - (xx[16] * xx[23] + xx[49] * xx[34]) * xx[8] + xx[1];
  xx[71] = xx[84] + xx[8] * (xx[16] * xx[15] - xx[34] * xx[6]);
  xx[72] = xx[85] + (xx[16] * xx[6] + xx[34] * xx[15]) * xx[8];
  pm_math_Quaternion_xform_ra(xx + 66, xx + 70, xx + 80);
  xx[66] = motionData[175];
  xx[67] = motionData[176];
  xx[68] = motionData[177];
  xx[69] = motionData[178];
  xx[70] = xx[4] * state[88] + xx[5] * state[87];
  xx[71] = - (xx[4] * state[87] - xx[5] * state[88]);
  xx[72] = xx[4] * state[90] + xx[5] * state[89];
  xx[73] = - (xx[4] * state[89] - xx[5] * state[90]);
  pm_math_Quaternion_compose_ra(xx + 66, xx + 70, xx + 83);
  xx[1] = xx[21] * xx[85];
  xx[4] = xx[22] * xx[86];
  xx[5] = xx[1] + xx[4];
  xx[6] = xx[21] * xx[84];
  xx[66] = xx[5];
  xx[67] = - xx[6];
  xx[68] = - (xx[22] * xx[84]);
  pm_math_Vector3_cross_ra(xx + 84, xx + 66, xx + 69);
  xx[15] = xx[20] * xx[84];
  xx[16] = xx[4] + xx[15];
  xx[4] = xx[22] * xx[85];
  xx[66] = - (xx[20] * xx[85]);
  xx[67] = xx[16];
  xx[68] = - xx[4];
  pm_math_Vector3_cross_ra(xx + 84, xx + 66, xx + 87);
  xx[23] = xx[20] * xx[86];
  xx[34] = xx[15] + xx[1];
  xx[66] = - xx[23];
  xx[67] = - (xx[21] * xx[86]);
  xx[68] = xx[34];
  pm_math_Vector3_cross_ra(xx + 84, xx + 66, xx + 96);
  J[29] = xx[17] + (xx[25] * xx[3] + xx[31]) * xx[8] + (xx[3] * xx[35] + xx[41])
    * xx[8];
  J[30] = xx[44] + motionData[314] + xx[8] * (xx[50] - xx[7] * xx[3]) + (xx[48] *
    xx[3] + xx[57]) * xx[8] + xx[28];
  J[31] = xx[54] + xx[8] * (xx[38] - xx[2] * xx[3]) + xx[8] * (xx[63] + xx[9] *
    xx[3]) - motionData[313] + xx[30];
  J[35] = xx[74] + xx[60];
  J[36] = xx[90] + xx[93];
  J[37] = xx[77] + xx[80];
  J[38] = (xx[83] * xx[5] + xx[69]) * xx[8];
  J[39] = xx[22] + (xx[87] - xx[20] * xx[83] * xx[85]) * xx[8];
  J[40] = xx[8] * (xx[96] - xx[83] * xx[23]) - xx[21];
  J[87] = xx[18] + xx[8] * (xx[32] - xx[26] * xx[3]) + xx[8] * (xx[42] + xx[36] *
    xx[3]) - motionData[314] - xx[28];
  J[88] = xx[45] + (xx[10] * xx[3] + xx[51]) * xx[8] + (xx[53] * xx[3] + xx[58])
    * xx[8];
  J[89] = xx[55] + motionData[312] + xx[8] * (xx[39] - xx[11] * xx[3]) + (xx[13]
    * xx[3] + xx[64]) * xx[8] - xx[47];
  J[93] = xx[75] + xx[61];
  J[94] = xx[91] + xx[94];
  J[95] = xx[78] + xx[81];
  J[96] = xx[8] * (xx[70] - xx[83] * xx[6]) - xx[22];
  J[97] = (xx[83] * xx[16] + xx[88]) * xx[8];
  J[98] = xx[20] + (xx[97] - xx[21] * xx[83] * xx[86]) * xx[8];
  J[145] = xx[19] + motionData[313] + xx[8] * (xx[33] - xx[27] * xx[3]) + (xx[43]
    - xx[37] * xx[3]) * xx[8] - xx[30];
  J[146] = xx[46] + xx[8] * (xx[52] - xx[24] * xx[3]) + xx[8] * (xx[59] - xx[29]
    * xx[3]) - motionData[312] + xx[47];
  J[147] = xx[56] + (xx[12] * xx[3] + xx[40]) * xx[8] + (xx[65] - xx[3] * xx[14])
    * xx[8];
  J[151] = xx[76] + xx[62];
  J[152] = xx[92] + xx[95];
  J[153] = xx[79] + xx[82];
  J[154] = xx[21] + (xx[71] - xx[22] * xx[83] * xx[84]) * xx[8];
  J[155] = xx[8] * (xx[89] - xx[83] * xx[4]) - xx[20];
  J[156] = (xx[83] * xx[34] + xx[98]) * xx[8];
  return 3;
}

static size_t computeAssemblyJacobian_6(const RuntimeDerivedValuesBundle *rtdv,
  const double *state, const int *modeVector, const double *motionData, double
  *J)
{
  const double *rtdvd = rtdv->mDoubles.mValues;
  const int *rtdvi = rtdv->mInts.mValues;
  double xx[105];
  (void) rtdvd;
  (void) rtdvi;
  (void) modeVector;
  xx[0] = 0.0;
  xx[1] = 0.9998343934839863;
  xx[2] = 0.0181985056119827;
  xx[3] = xx[1] * state[66] + xx[2] * state[68];
  xx[4] = xx[1] * state[67] - xx[2] * state[69];
  xx[5] = xx[1] * state[68] - xx[2] * state[66];
  xx[6] = xx[1] * state[69] + xx[2] * state[67];
  xx[7] = motionData[308];
  xx[8] = motionData[309];
  xx[9] = motionData[310];
  xx[10] = motionData[311];
  pm_math_Quaternion_compose_ra(xx + 3, xx + 7, xx + 11);
  xx[1] = 1.0;
  xx[2] = motionData[310] * motionData[310];
  xx[7] = motionData[311] * motionData[311];
  xx[8] = 2.0;
  xx[9] = motionData[309] * motionData[310];
  xx[10] = motionData[308] * motionData[311];
  xx[15] = motionData[308] * motionData[310];
  xx[16] = motionData[309] * motionData[311];
  xx[17] = xx[1] - (xx[2] + xx[7]) * xx[8];
  xx[18] = xx[8] * (xx[9] - xx[10]);
  xx[19] = (xx[15] + xx[16]) * xx[8];
  xx[20] = 8.94e-3;
  xx[21] = 0.3143400000000001;
  xx[22] = 9.000000000000001e-3;
  xx[23] = - xx[20];
  xx[24] = xx[21];
  xx[25] = xx[22];
  pm_math_Vector3_cross_ra(xx + 17, xx + 23, xx + 26);
  pm_math_Quaternion_xform_ra(xx + 11, xx + 26, xx + 17);
  xx[26] = xx[5] * motionData[313];
  xx[27] = xx[6] * motionData[314];
  xx[28] = xx[26] + xx[27];
  xx[29] = xx[4] * motionData[313];
  xx[30] = xx[4] * motionData[314];
  xx[31] = xx[28];
  xx[32] = - xx[29];
  xx[33] = - xx[30];
  pm_math_Vector3_cross_ra(xx + 4, xx + 31, xx + 34);
  xx[31] = 0.01433000000000001;
  xx[32] = xx[31] * xx[6];
  xx[33] = 0.284;
  xx[37] = xx[5] * xx[33];
  xx[38] = xx[32] - xx[37];
  xx[39] = xx[4] * xx[33];
  xx[40] = xx[4] * xx[31];
  xx[41] = xx[38];
  xx[42] = xx[39];
  xx[43] = - xx[40];
  pm_math_Vector3_cross_ra(xx + 4, xx + 41, xx + 44);
  xx[41] = motionData[309] * motionData[309];
  xx[42] = motionData[310] * motionData[311];
  xx[43] = motionData[308] * motionData[309];
  xx[47] = (xx[10] + xx[9]) * xx[8];
  xx[48] = xx[1] - (xx[7] + xx[41]) * xx[8];
  xx[49] = xx[8] * (xx[42] - xx[43]);
  pm_math_Vector3_cross_ra(xx + 47, xx + 23, xx + 50);
  pm_math_Quaternion_xform_ra(xx + 11, xx + 50, xx + 47);
  xx[7] = xx[5] * motionData[312];
  xx[9] = xx[4] * motionData[312];
  xx[10] = xx[27] + xx[9];
  xx[27] = xx[5] * motionData[314];
  xx[50] = - xx[7];
  xx[51] = xx[10];
  xx[52] = - xx[27];
  pm_math_Vector3_cross_ra(xx + 4, xx + 50, xx + 53);
  xx[50] = 0.5109;
  xx[51] = xx[5] * xx[50];
  xx[52] = xx[4] * xx[50];
  xx[56] = xx[32] - xx[52];
  xx[32] = xx[5] * xx[31];
  xx[57] = xx[51];
  xx[58] = xx[56];
  xx[59] = - xx[32];
  pm_math_Vector3_cross_ra(xx + 4, xx + 57, xx + 60);
  xx[57] = xx[8] * (xx[16] - xx[15]);
  xx[58] = (xx[43] + xx[42]) * xx[8];
  xx[59] = xx[1] - (xx[41] + xx[2]) * xx[8];
  pm_math_Vector3_cross_ra(xx + 57, xx + 23, xx + 41);
  pm_math_Quaternion_xform_ra(xx + 11, xx + 41, xx + 57);
  xx[2] = xx[6] * motionData[312];
  xx[11] = xx[6] * motionData[313];
  xx[12] = xx[9] + xx[26];
  xx[13] = - xx[2];
  xx[14] = - xx[11];
  xx[15] = xx[12];
  pm_math_Vector3_cross_ra(xx + 4, xx + 13, xx + 41);
  xx[9] = xx[50] * xx[6];
  xx[13] = xx[33] * xx[6];
  xx[14] = xx[52] + xx[37];
  xx[63] = xx[9];
  xx[64] = xx[13];
  xx[65] = - xx[14];
  pm_math_Vector3_cross_ra(xx + 4, xx + 63, xx + 66);
  xx[69] = motionData[70];
  xx[70] = motionData[71];
  xx[71] = motionData[72];
  xx[72] = motionData[73];
  xx[4] = 2.960959925763799e-4;
  xx[5] = 0.9999999561635808;
  xx[6] = xx[4] * state[81] - xx[5] * state[80];
  xx[15] = xx[4] * state[80] + xx[5] * state[81];
  xx[16] = - xx[15];
  xx[26] = xx[4] * state[83] + xx[5] * state[82];
  xx[37] = - xx[26];
  xx[52] = xx[5] * state[83] - xx[4] * state[82];
  xx[63] = - xx[52];
  xx[73] = xx[6];
  xx[74] = xx[16];
  xx[75] = xx[37];
  xx[76] = xx[63];
  pm_math_Quaternion_compose_ra(xx + 69, xx + 73, xx + 77);
  xx[81] = motionData[91];
  xx[82] = motionData[92];
  xx[83] = motionData[93];
  xx[84] = motionData[94];
  pm_math_Quaternion_compose_ra(xx + 77, xx + 81, xx + 85);
  xx[77] = xx[1] - (motionData[93] * motionData[93] + motionData[94] *
                    motionData[94]) * xx[8];
  xx[78] = xx[8] * (motionData[92] * motionData[93] - motionData[91] *
                    motionData[94]);
  xx[79] = (motionData[91] * motionData[93] + motionData[92] * motionData[94]) *
    xx[8];
  pm_math_Vector3_cross_ra(xx + 77, xx + 23, xx + 80);
  pm_math_Quaternion_xform_ra(xx + 85, xx + 80, xx + 77);
  xx[80] = xx[16];
  xx[81] = xx[37];
  xx[82] = xx[63];
  xx[1] = xx[26] * motionData[96] + xx[52] * motionData[97];
  xx[16] = xx[15] * motionData[96];
  xx[37] = xx[15] * motionData[97];
  xx[63] = - xx[1];
  xx[64] = xx[16];
  xx[65] = xx[37];
  pm_math_Vector3_cross_ra(xx + 80, xx + 63, xx + 89);
  xx[63] = 0.09037053171319967;
  xx[64] = 0.0213683169329786;
  xx[65] = xx[63] * xx[26] + xx[52] * xx[64];
  xx[83] = xx[63] * xx[15];
  xx[84] = xx[64] * xx[15];
  xx[92] = xx[65];
  xx[93] = - xx[83];
  xx[94] = - xx[84];
  pm_math_Vector3_cross_ra(xx + 80, xx + 92, xx + 95);
  xx[80] = (xx[89] - xx[1] * xx[6]) * xx[8] + (xx[65] * xx[6] + xx[95]) * xx[8];
  xx[81] = xx[8] * (xx[90] + xx[16] * xx[6]) + xx[8] * (xx[96] - xx[83] * xx[6])
    - motionData[97] + xx[64];
  xx[82] = motionData[96] + xx[8] * (xx[91] + xx[37] * xx[6]) + (xx[97] - xx[84]
    * xx[6]) * xx[8] - xx[63];
  pm_math_Quaternion_xform_ra(xx + 69, xx + 80, xx + 63);
  xx[80] = motionData[92];
  xx[81] = motionData[93];
  xx[82] = motionData[94];
  xx[1] = 5.921919591931837e-4;
  xx[16] = 0.9999998246543264;
  xx[37] = xx[1] * motionData[93] + xx[16] * motionData[94];
  xx[83] = xx[1] * motionData[92];
  xx[84] = xx[16] * motionData[92];
  xx[89] = - xx[37];
  xx[90] = xx[83];
  xx[91] = xx[84];
  pm_math_Vector3_cross_ra(xx + 80, xx + 89, xx + 92);
  xx[89] = motionData[91] * motionData[92];
  xx[95] = xx[8] * (xx[92] + xx[37] * motionData[91]);
  xx[96] = xx[16] + (xx[93] - xx[83] * motionData[91]) * xx[8];
  xx[97] = xx[8] * (xx[94] - xx[16] * xx[89]) - xx[1];
  pm_math_Vector3_cross_ra(xx + 95, xx + 23, xx + 90);
  pm_math_Quaternion_xform_ra(xx + 85, xx + 90, xx + 93);
  xx[37] = - (xx[1] * motionData[95]);
  xx[90] = xx[16] * motionData[95];
  xx[96] = xx[16] * motionData[97] + xx[1] * motionData[96];
  xx[97] = xx[37];
  xx[98] = - xx[90];
  pm_math_Quaternion_xform_ra(xx + 73, xx + 96, xx + 99);
  xx[91] = 0.02142182988836524;
  xx[92] = xx[91] * xx[26];
  xx[96] = xx[52] * xx[91];
  xx[102] = xx[99] + (xx[92] * xx[26] + xx[52] * xx[96]) * xx[8] - xx[91];
  xx[103] = xx[100] + xx[8] * (xx[96] * xx[6] - xx[92] * xx[15]);
  xx[104] = xx[101] - (xx[92] * xx[6] + xx[96] * xx[15]) * xx[8];
  pm_math_Quaternion_xform_ra(xx + 69, xx + 102, xx + 96);
  xx[91] = xx[16] * motionData[93] - xx[1] * motionData[94];
  xx[99] = xx[91];
  xx[100] = - xx[84];
  xx[101] = xx[83];
  pm_math_Vector3_cross_ra(xx + 80, xx + 99, xx + 102);
  xx[80] = xx[8] * (xx[102] - xx[91] * motionData[91]);
  xx[81] = xx[1] + (xx[84] * motionData[91] + xx[103]) * xx[8];
  xx[82] = xx[16] + xx[8] * (xx[104] - xx[1] * xx[89]);
  pm_math_Vector3_cross_ra(xx + 80, xx + 23, xx + 99);
  pm_math_Quaternion_xform_ra(xx + 85, xx + 99, xx + 23);
  xx[80] = xx[1] * motionData[97] - xx[16] * motionData[96];
  xx[81] = xx[90];
  xx[82] = xx[37];
  pm_math_Quaternion_xform_ra(xx + 73, xx + 80, xx + 83);
  xx[1] = 0.0903578617216487;
  xx[16] = xx[1] * xx[26];
  xx[37] = xx[52] * xx[1];
  xx[73] = xx[83] - (xx[16] * xx[26] + xx[52] * xx[37]) * xx[8] + xx[1];
  xx[74] = xx[84] + xx[8] * (xx[16] * xx[15] - xx[37] * xx[6]);
  xx[75] = xx[85] + (xx[16] * xx[6] + xx[37] * xx[15]) * xx[8];
  pm_math_Quaternion_xform_ra(xx + 69, xx + 73, xx + 80);
  xx[69] = motionData[175];
  xx[70] = motionData[176];
  xx[71] = motionData[177];
  xx[72] = motionData[178];
  xx[73] = xx[4] * state[88] + xx[5] * state[87];
  xx[74] = - (xx[4] * state[87] - xx[5] * state[88]);
  xx[75] = xx[4] * state[90] + xx[5] * state[89];
  xx[76] = - (xx[4] * state[89] - xx[5] * state[90]);
  pm_math_Quaternion_compose_ra(xx + 69, xx + 73, xx + 83);
  xx[1] = xx[21] * xx[85];
  xx[4] = xx[22] * xx[86];
  xx[5] = xx[1] + xx[4];
  xx[6] = xx[21] * xx[84];
  xx[69] = xx[5];
  xx[70] = - xx[6];
  xx[71] = - (xx[22] * xx[84]);
  pm_math_Vector3_cross_ra(xx + 84, xx + 69, xx + 72);
  xx[15] = xx[20] * xx[84];
  xx[16] = xx[4] - xx[15];
  xx[4] = xx[22] * xx[85];
  xx[69] = xx[20] * xx[85];
  xx[70] = xx[16];
  xx[71] = - xx[4];
  pm_math_Vector3_cross_ra(xx + 84, xx + 69, xx + 87);
  xx[26] = xx[20] * xx[86];
  xx[37] = xx[1] - xx[15];
  xx[69] = xx[26];
  xx[70] = - (xx[21] * xx[86]);
  xx[71] = xx[37];
  pm_math_Vector3_cross_ra(xx + 84, xx + 69, xx + 90);
  J[29] = xx[17] + (xx[28] * xx[3] + xx[34]) * xx[8] + (xx[3] * xx[38] + xx[44])
    * xx[8];
  J[30] = xx[47] + motionData[314] + xx[8] * (xx[53] - xx[7] * xx[3]) + (xx[51] *
    xx[3] + xx[60]) * xx[8] + xx[31];
  J[31] = xx[57] + xx[8] * (xx[41] - xx[2] * xx[3]) + xx[8] * (xx[66] + xx[9] *
    xx[3]) - motionData[313] + xx[33];
  J[35] = xx[77] + xx[63];
  J[36] = xx[93] + xx[96];
  J[37] = xx[23] + xx[80];
  J[38] = (xx[83] * xx[5] + xx[72]) * xx[8];
  J[39] = xx[22] + (xx[20] * xx[83] * xx[85] + xx[87]) * xx[8];
  J[40] = xx[8] * (xx[90] + xx[83] * xx[26]) - xx[21];
  J[87] = xx[18] + xx[8] * (xx[35] - xx[29] * xx[3]) + xx[8] * (xx[45] + xx[39] *
    xx[3]) - motionData[314] - xx[31];
  J[88] = xx[48] + (xx[10] * xx[3] + xx[54]) * xx[8] + (xx[56] * xx[3] + xx[61])
    * xx[8];
  J[89] = xx[58] + motionData[312] + xx[8] * (xx[42] - xx[11] * xx[3]) + (xx[13]
    * xx[3] + xx[67]) * xx[8] - xx[50];
  J[93] = xx[78] + xx[64];
  J[94] = xx[94] + xx[97];
  J[95] = xx[24] + xx[81];
  J[96] = xx[8] * (xx[73] - xx[83] * xx[6]) - xx[22];
  J[97] = (xx[83] * xx[16] + xx[88]) * xx[8];
  J[98] = (xx[91] - xx[21] * xx[83] * xx[86]) * xx[8] - xx[20];
  J[145] = xx[19] + motionData[313] + xx[8] * (xx[36] - xx[30] * xx[3]) + (xx[46]
    - xx[40] * xx[3]) * xx[8] - xx[33];
  J[146] = xx[49] + xx[8] * (xx[55] - xx[27] * xx[3]) + xx[8] * (xx[62] - xx[32]
    * xx[3]) - motionData[312] + xx[50];
  J[147] = xx[59] + (xx[12] * xx[3] + xx[43]) * xx[8] + (xx[68] - xx[3] * xx[14])
    * xx[8];
  J[151] = xx[79] + xx[65];
  J[152] = xx[95] + xx[98];
  J[153] = xx[25] + xx[82];
  J[154] = xx[21] + (xx[74] - xx[22] * xx[83] * xx[84]) * xx[8];
  J[155] = xx[20] + xx[8] * (xx[89] - xx[83] * xx[4]);
  J[156] = (xx[83] * xx[37] + xx[92]) * xx[8];
  return 3;
}

static size_t computeAssemblyJacobian_7(const RuntimeDerivedValuesBundle *rtdv,
  const double *state, const int *modeVector, const double *motionData, double
  *J)
{
  const double *rtdvd = rtdv->mDoubles.mValues;
  const int *rtdvi = rtdv->mInts.mValues;
  double xx[51];
  (void) rtdvd;
  (void) rtdvi;
  (void) modeVector;
  (void) motionData;
  xx[0] = 0.0;
  xx[1] = 0.9998343934839863;
  xx[2] = 0.0181985056119827;
  xx[3] = xx[1] * state[38] + xx[2] * state[40];
  xx[4] = 9.000000000000001e-3;
  xx[5] = xx[1] * state[41] + xx[2] * state[39];
  xx[6] = xx[4] * xx[5];
  xx[7] = xx[1] * state[40] - xx[2] * state[38];
  xx[8] = 0.3143400000000001;
  xx[9] = xx[7] * xx[8];
  xx[10] = xx[6] - xx[9];
  xx[11] = xx[1] * state[39] - xx[2] * state[41];
  xx[12] = xx[11];
  xx[13] = xx[7];
  xx[14] = xx[5];
  xx[1] = xx[11] * xx[8];
  xx[2] = xx[11] * xx[4];
  xx[15] = xx[10];
  xx[16] = xx[1];
  xx[17] = - xx[2];
  pm_math_Vector3_cross_ra(xx + 12, xx + 15, xx + 18);
  xx[15] = 2.0;
  xx[16] = 0.27424;
  xx[17] = xx[7] * xx[16];
  xx[21] = 8.0e-3;
  xx[22] = xx[21] * xx[5];
  xx[23] = xx[17] - xx[22];
  xx[24] = xx[11] * xx[16];
  xx[25] = xx[11] * xx[21];
  xx[26] = xx[23];
  xx[27] = - xx[24];
  xx[28] = xx[25];
  pm_math_Vector3_cross_ra(xx + 12, xx + 26, xx + 29);
  xx[26] = 8.94e-3;
  xx[27] = xx[7] * xx[26];
  xx[28] = xx[11] * xx[26];
  xx[32] = xx[6] - xx[28];
  xx[6] = xx[7] * xx[4];
  xx[33] = xx[27];
  xx[34] = xx[32];
  xx[35] = - xx[6];
  pm_math_Vector3_cross_ra(xx + 12, xx + 33, xx + 36);
  xx[4] = 0.513;
  xx[33] = xx[7] * xx[4];
  xx[34] = xx[11] * xx[4];
  xx[11] = xx[22] + xx[34];
  xx[22] = xx[7] * xx[21];
  xx[39] = xx[33];
  xx[40] = - xx[11];
  xx[41] = xx[22];
  pm_math_Vector3_cross_ra(xx + 12, xx + 39, xx + 42);
  xx[7] = 1.000000000000001e-3;
  xx[21] = xx[26] * xx[5];
  xx[26] = xx[8] * xx[5];
  xx[8] = xx[28] + xx[9];
  xx[39] = xx[21];
  xx[40] = xx[26];
  xx[41] = - xx[8];
  pm_math_Vector3_cross_ra(xx + 12, xx + 39, xx + 45);
  xx[9] = xx[4] * xx[5];
  xx[4] = xx[16] * xx[5];
  xx[5] = xx[17] - xx[34];
  xx[39] = xx[9];
  xx[40] = - xx[4];
  xx[41] = xx[5];
  pm_math_Vector3_cross_ra(xx + 12, xx + 39, xx + 48);
  xx[12] = 0.04010000000000002;
  xx[13] = 0.52194;
  J[17] = (xx[3] * xx[10] + xx[18]) * xx[15] + (xx[23] * xx[3] + xx[29]) * xx[15];
  J[18] = (xx[27] * xx[3] + xx[36]) * xx[15] + (xx[33] * xx[3] + xx[42]) * xx[15]
    + xx[7];
  J[19] = xx[15] * (xx[45] + xx[21] * xx[3]) + xx[15] * (xx[48] + xx[9] * xx[3])
    + xx[12];
  J[75] = xx[15] * (xx[19] + xx[1] * xx[3]) + xx[15] * (xx[30] - xx[24] * xx[3])
    - xx[7];
  J[76] = (xx[32] * xx[3] + xx[37]) * xx[15] + (xx[43] - xx[3] * xx[11]) * xx[15];
  J[77] = (xx[26] * xx[3] + xx[46]) * xx[15] + (xx[49] - xx[4] * xx[3]) * xx[15]
    - xx[13];
  J[133] = (xx[20] - xx[2] * xx[3]) * xx[15] + (xx[25] * xx[3] + xx[31]) * xx[15]
    - xx[12];
  J[134] = xx[15] * (xx[38] - xx[6] * xx[3]) + xx[15] * (xx[44] + xx[22] * xx[3])
    + xx[13];
  J[135] = (xx[47] - xx[3] * xx[8]) * xx[15] + (xx[3] * xx[5] + xx[50]) * xx[15];
  return 3;
}

static size_t computeAssemblyJacobian_8(const RuntimeDerivedValuesBundle *rtdv,
  const double *state, const int *modeVector, const double *motionData, double
  *J)
{
  const double *rtdvd = rtdv->mDoubles.mValues;
  const int *rtdvi = rtdv->mInts.mValues;
  double xx[111];
  (void) rtdvd;
  (void) rtdvi;
  (void) modeVector;
  xx[0] = 0.0;
  xx[1] = 0.9998343934839863;
  xx[2] = 0.0181985056119827;
  xx[3] = xx[1] * state[38] + xx[2] * state[40];
  xx[4] = xx[1] * state[39] - xx[2] * state[41];
  xx[5] = xx[1] * state[40] - xx[2] * state[38];
  xx[6] = xx[1] * state[41] + xx[2] * state[39];
  xx[7] = motionData[315];
  xx[8] = motionData[316];
  xx[9] = motionData[317];
  xx[10] = motionData[318];
  pm_math_Quaternion_compose_ra(xx + 3, xx + 7, xx + 11);
  xx[1] = 1.0;
  xx[2] = motionData[317] * motionData[317];
  xx[7] = motionData[318] * motionData[318];
  xx[8] = 2.0;
  xx[9] = motionData[316] * motionData[317];
  xx[10] = motionData[315] * motionData[318];
  xx[15] = motionData[315] * motionData[317];
  xx[16] = motionData[316] * motionData[318];
  xx[17] = xx[1] - (xx[2] + xx[7]) * xx[8];
  xx[18] = xx[8] * (xx[9] - xx[10]);
  xx[19] = (xx[15] + xx[16]) * xx[8];
  xx[20] = 0.5109;
  xx[21] = - 0.284;
  xx[22] = - 0.01433000000000001;
  pm_math_Vector3_cross_ra(xx + 17, xx + 20, xx + 23);
  pm_math_Quaternion_xform_ra(xx + 11, xx + 23, xx + 17);
  xx[23] = xx[5] * motionData[320];
  xx[24] = xx[6] * motionData[321];
  xx[25] = xx[23] + xx[24];
  xx[26] = xx[4] * motionData[320];
  xx[27] = xx[4] * motionData[321];
  xx[28] = xx[25];
  xx[29] = - xx[26];
  xx[30] = - xx[27];
  pm_math_Vector3_cross_ra(xx + 4, xx + 28, xx + 31);
  xx[28] = 0.27424;
  xx[29] = xx[5] * xx[28];
  xx[30] = 8.0e-3;
  xx[34] = xx[30] * xx[6];
  xx[35] = xx[29] - xx[34];
  xx[36] = xx[4] * xx[28];
  xx[37] = xx[4] * xx[30];
  xx[38] = xx[35];
  xx[39] = - xx[36];
  xx[40] = xx[37];
  pm_math_Vector3_cross_ra(xx + 4, xx + 38, xx + 41);
  xx[38] = motionData[316] * motionData[316];
  xx[39] = motionData[317] * motionData[318];
  xx[40] = motionData[315] * motionData[316];
  xx[44] = (xx[10] + xx[9]) * xx[8];
  xx[45] = xx[1] - (xx[7] + xx[38]) * xx[8];
  xx[46] = xx[8] * (xx[39] - xx[40]);
  pm_math_Vector3_cross_ra(xx + 44, xx + 20, xx + 47);
  pm_math_Quaternion_xform_ra(xx + 11, xx + 47, xx + 44);
  xx[7] = xx[5] * motionData[319];
  xx[9] = xx[4] * motionData[319];
  xx[10] = xx[24] + xx[9];
  xx[24] = xx[5] * motionData[321];
  xx[47] = - xx[7];
  xx[48] = xx[10];
  xx[49] = - xx[24];
  pm_math_Vector3_cross_ra(xx + 4, xx + 47, xx + 50);
  xx[47] = 0.513;
  xx[48] = xx[5] * xx[47];
  xx[49] = xx[4] * xx[47];
  xx[53] = xx[34] + xx[49];
  xx[34] = xx[5] * xx[30];
  xx[54] = xx[48];
  xx[55] = - xx[53];
  xx[56] = xx[34];
  pm_math_Vector3_cross_ra(xx + 4, xx + 54, xx + 57);
  xx[54] = xx[8] * (xx[16] - xx[15]);
  xx[55] = (xx[40] + xx[39]) * xx[8];
  xx[56] = xx[1] - (xx[38] + xx[2]) * xx[8];
  pm_math_Vector3_cross_ra(xx + 54, xx + 20, xx + 38);
  pm_math_Quaternion_xform_ra(xx + 11, xx + 38, xx + 54);
  xx[2] = xx[6] * motionData[319];
  xx[11] = xx[6] * motionData[320];
  xx[12] = xx[9] + xx[23];
  xx[13] = - xx[2];
  xx[14] = - xx[11];
  xx[15] = xx[12];
  pm_math_Vector3_cross_ra(xx + 4, xx + 13, xx + 38);
  xx[9] = xx[47] * xx[6];
  xx[13] = xx[28] * xx[6];
  xx[14] = xx[29] - xx[49];
  xx[60] = xx[9];
  xx[61] = - xx[13];
  xx[62] = xx[14];
  pm_math_Vector3_cross_ra(xx + 4, xx + 60, xx + 63);
  xx[66] = motionData[42];
  xx[67] = motionData[43];
  xx[68] = motionData[44];
  xx[69] = motionData[45];
  xx[4] = 0.9999999561635808;
  xx[5] = 2.960959925766019e-4;
  xx[6] = xx[4] * state[45] - xx[5] * state[46];
  xx[15] = xx[5] * state[45] + xx[4] * state[46];
  xx[16] = xx[4] * state[47] - xx[5] * state[48];
  xx[23] = xx[4] * state[48] + xx[5] * state[47];
  xx[70] = - xx[6];
  xx[71] = xx[15];
  xx[72] = xx[16];
  xx[73] = xx[23];
  pm_math_Quaternion_compose_ra(xx + 66, xx + 70, xx + 74);
  xx[78] = motionData[56];
  xx[79] = motionData[57];
  xx[80] = motionData[58];
  xx[81] = motionData[59];
  pm_math_Quaternion_compose_ra(xx + 74, xx + 78, xx + 82);
  xx[29] = xx[16] * xx[16];
  xx[49] = xx[23] * xx[23];
  xx[60] = (xx[29] + xx[49]) * xx[8] - xx[1];
  xx[61] = xx[6] * xx[23];
  xx[62] = xx[15] * xx[16];
  xx[74] = xx[61] + xx[62];
  xx[75] = xx[6] * xx[16];
  xx[76] = xx[15] * xx[23];
  xx[77] = xx[75] - xx[76];
  xx[86] = xx[60];
  xx[87] = - (xx[74] * xx[8]);
  xx[88] = xx[8] * xx[77];
  pm_math_Quaternion_inverseXform_ra(xx + 78, xx + 86, xx + 89);
  pm_math_Vector3_cross_ra(xx + 89, xx + 20, xx + 92);
  pm_math_Quaternion_xform_ra(xx + 82, xx + 92, xx + 89);
  xx[92] = motionData[60];
  xx[93] = motionData[61];
  xx[94] = motionData[62];
  pm_math_Vector3_cross_ra(xx + 86, xx + 92, xx + 95);
  pm_math_Quaternion_xform_ra(xx + 70, xx + 95, xx + 86);
  xx[95] = 0.283893415005525;
  xx[96] = 0.1300676268524588;
  xx[97] = 0.1419467075027625;
  xx[98] = 0.06503381342622938;
  xx[99] = - (xx[74] * xx[95] + xx[96] * xx[77]);
  xx[100] = - (xx[60] * xx[97]);
  xx[101] = xx[60] * xx[98];
  pm_math_Quaternion_xform_ra(xx + 70, xx + 99, xx + 102);
  xx[99] = xx[86] + xx[102];
  xx[100] = xx[87] + xx[103];
  xx[101] = xx[88] + xx[104];
  pm_math_Quaternion_xform_ra(xx + 66, xx + 99, xx + 86);
  xx[60] = xx[8] * (xx[61] - xx[62]);
  xx[61] = xx[15] * xx[15];
  xx[62] = (xx[49] + xx[61]) * xx[8] - xx[1];
  xx[49] = xx[6] * xx[15];
  xx[6] = xx[23] * xx[16];
  xx[15] = xx[49] + xx[6];
  xx[99] = xx[60];
  xx[100] = xx[62];
  xx[101] = - (xx[15] * xx[8]);
  pm_math_Quaternion_inverseXform_ra(xx + 78, xx + 99, xx + 102);
  pm_math_Vector3_cross_ra(xx + 102, xx + 20, xx + 105);
  pm_math_Quaternion_xform_ra(xx + 82, xx + 105, xx + 102);
  pm_math_Vector3_cross_ra(xx + 99, xx + 92, xx + 105);
  pm_math_Quaternion_xform_ra(xx + 70, xx + 105, xx + 99);
  xx[105] = xx[62] * xx[97] + xx[15] * xx[96];
  xx[106] = - (xx[97] * xx[60]);
  xx[107] = xx[98] * xx[60];
  pm_math_Quaternion_xform_ra(xx + 70, xx + 105, xx + 108);
  xx[105] = xx[99] + xx[108];
  xx[106] = xx[100] + xx[109];
  xx[107] = xx[101] + xx[110];
  pm_math_Quaternion_xform_ra(xx + 66, xx + 105, xx + 99);
  xx[15] = (xx[75] + xx[76]) * xx[8];
  xx[16] = xx[49] - xx[6];
  xx[6] = (xx[61] + xx[29]) * xx[8] - xx[1];
  xx[60] = - xx[15];
  xx[61] = xx[8] * xx[16];
  xx[62] = xx[6];
  pm_math_Quaternion_inverseXform_ra(xx + 78, xx + 60, xx + 74);
  pm_math_Vector3_cross_ra(xx + 74, xx + 20, xx + 77);
  pm_math_Quaternion_xform_ra(xx + 82, xx + 77, xx + 74);
  pm_math_Vector3_cross_ra(xx + 60, xx + 92, xx + 77);
  pm_math_Quaternion_xform_ra(xx + 70, xx + 77, xx + 60);
  xx[77] = xx[95] * xx[16] - xx[6] * xx[98];
  xx[78] = xx[97] * xx[15];
  xx[79] = - (xx[98] * xx[15]);
  pm_math_Quaternion_xform_ra(xx + 70, xx + 77, xx + 80);
  xx[70] = xx[60] + xx[80];
  xx[71] = xx[61] + xx[81];
  xx[72] = xx[62] + xx[82];
  pm_math_Quaternion_xform_ra(xx + 66, xx + 70, xx + 60);
  xx[66] = motionData[210];
  xx[67] = motionData[211];
  xx[68] = motionData[212];
  xx[69] = motionData[213];
  xx[6] = xx[4] * state[52] + xx[5] * state[53];
  xx[15] = xx[5] * state[52] - xx[4] * state[53];
  xx[16] = xx[5] * state[55] - xx[4] * state[54];
  xx[23] = xx[4] * state[55] + xx[5] * state[54];
  xx[4] = - xx[23];
  xx[70] = xx[6];
  xx[71] = xx[15];
  xx[72] = xx[16];
  xx[73] = xx[4];
  pm_math_Quaternion_compose_ra(xx + 66, xx + 70, xx + 77);
  xx[66] = (xx[16] * xx[16] + xx[23] * xx[23]) * xx[8] - xx[1];
  xx[67] = - ((xx[6] * xx[23] + xx[15] * xx[16]) * xx[8]);
  xx[68] = xx[8] * (xx[23] * xx[15] - xx[6] * xx[16]);
  pm_math_Vector3_cross_ra(xx + 66, xx + 20, xx + 69);
  pm_math_Quaternion_xform_ra(xx + 77, xx + 69, xx + 66);
  xx[69] = xx[15];
  xx[70] = xx[16];
  xx[71] = xx[4];
  xx[1] = 5.921919591936278e-4;
  xx[4] = 0.9999998246543264;
  xx[5] = xx[1] * xx[16] + xx[23] * xx[4];
  xx[29] = xx[1] * xx[15];
  xx[49] = xx[4] * xx[15];
  xx[81] = - xx[5];
  xx[82] = xx[29];
  xx[83] = - xx[49];
  pm_math_Vector3_cross_ra(xx + 69, xx + 81, xx + 92);
  xx[15] = xx[6] * xx[29];
  xx[72] = xx[6] * xx[49];
  xx[81] = xx[8] * (xx[92] + xx[6] * xx[5]);
  xx[82] = (xx[93] - xx[15]) * xx[8] - xx[4];
  xx[83] = xx[8] * (xx[94] + xx[72]) - xx[1];
  pm_math_Vector3_cross_ra(xx + 81, xx + 20, xx + 92);
  pm_math_Quaternion_xform_ra(xx + 77, xx + 92, xx + 81);
  xx[5] = xx[23] * xx[1] - xx[4] * xx[16];
  xx[92] = xx[5];
  xx[93] = xx[49];
  xx[94] = xx[29];
  pm_math_Vector3_cross_ra(xx + 69, xx + 92, xx + 95);
  xx[69] = xx[8] * (xx[95] - xx[6] * xx[5]);
  xx[70] = xx[1] + (xx[96] - xx[72]) * xx[8];
  xx[71] = xx[8] * (xx[97] - xx[15]) - xx[4];
  pm_math_Vector3_cross_ra(xx + 69, xx + 20, xx + 4);
  pm_math_Quaternion_xform_ra(xx + 77, xx + 4, xx + 20);
  J[17] = xx[17] + (xx[25] * xx[3] + xx[31]) * xx[8] + (xx[35] * xx[3] + xx[41])
    * xx[8];
  J[18] = xx[44] + motionData[321] + xx[8] * (xx[50] - xx[7] * xx[3]) + (xx[48] *
    xx[3] + xx[57]) * xx[8] - xx[30];
  J[19] = xx[54] + xx[8] * (xx[38] - xx[2] * xx[3]) + xx[8] * (xx[63] + xx[9] *
    xx[3]) - motionData[320] - xx[28];
  J[20] = xx[89] + xx[86];
  J[21] = xx[102] + xx[99];
  J[22] = xx[74] + xx[60];
  J[23] = xx[66];
  J[24] = xx[81];
  J[25] = xx[20];
  J[75] = xx[18] + xx[8] * (xx[32] - xx[26] * xx[3]) + xx[8] * (xx[42] - xx[36] *
    xx[3]) - motionData[321] + xx[30];
  J[76] = xx[45] + (xx[10] * xx[3] + xx[51]) * xx[8] + (xx[58] - xx[3] * xx[53])
    * xx[8];
  J[77] = xx[55] + motionData[319] + xx[8] * (xx[39] - xx[11] * xx[3]) + (xx[64]
    - xx[13] * xx[3]) * xx[8] - xx[47];
  J[78] = xx[90] + xx[87];
  J[79] = xx[103] + xx[100];
  J[80] = xx[75] + xx[61];
  J[81] = xx[67];
  J[82] = xx[82];
  J[83] = xx[21];
  J[133] = xx[19] + motionData[320] + xx[8] * (xx[33] - xx[27] * xx[3]) + (xx[37]
    * xx[3] + xx[43]) * xx[8] + xx[28];
  J[134] = xx[46] + xx[8] * (xx[52] - xx[24] * xx[3]) + xx[8] * (xx[59] + xx[34]
    * xx[3]) - motionData[319] + xx[47];
  J[135] = xx[56] + (xx[12] * xx[3] + xx[40]) * xx[8] + (xx[3] * xx[14] + xx[65])
    * xx[8];
  J[136] = xx[91] + xx[88];
  J[137] = xx[104] + xx[101];
  J[138] = xx[76] + xx[62];
  J[139] = xx[68];
  J[140] = xx[83];
  J[141] = xx[22];
  return 3;
}

static size_t computeAssemblyJacobian_9(const RuntimeDerivedValuesBundle *rtdv,
  const double *state, const int *modeVector, const double *motionData, double
  *J)
{
  const double *rtdvd = rtdv->mDoubles.mValues;
  const int *rtdvi = rtdv->mInts.mValues;
  double xx[111];
  (void) rtdvd;
  (void) rtdvi;
  (void) modeVector;
  xx[0] = 0.0;
  xx[1] = 0.9998343934839863;
  xx[2] = 0.0181985056119827;
  xx[3] = xx[1] * state[38] + xx[2] * state[40];
  xx[4] = xx[1] * state[39] - xx[2] * state[41];
  xx[5] = xx[1] * state[40] - xx[2] * state[38];
  xx[6] = xx[1] * state[41] + xx[2] * state[39];
  xx[7] = motionData[315];
  xx[8] = motionData[316];
  xx[9] = motionData[317];
  xx[10] = motionData[318];
  pm_math_Quaternion_compose_ra(xx + 3, xx + 7, xx + 11);
  xx[1] = 1.0;
  xx[2] = motionData[317] * motionData[317];
  xx[7] = motionData[318] * motionData[318];
  xx[8] = 2.0;
  xx[9] = motionData[316] * motionData[317];
  xx[10] = motionData[315] * motionData[318];
  xx[15] = motionData[315] * motionData[317];
  xx[16] = motionData[316] * motionData[318];
  xx[17] = xx[1] - (xx[2] + xx[7]) * xx[8];
  xx[18] = xx[8] * (xx[9] - xx[10]);
  xx[19] = (xx[15] + xx[16]) * xx[8];
  xx[20] = - 8.82e-3;
  xx[21] = - 0.2541;
  xx[22] = - 0.01433000000000001;
  pm_math_Vector3_cross_ra(xx + 17, xx + 20, xx + 23);
  pm_math_Quaternion_xform_ra(xx + 11, xx + 23, xx + 17);
  xx[23] = xx[5] * motionData[320];
  xx[24] = xx[6] * motionData[321];
  xx[25] = xx[23] + xx[24];
  xx[26] = xx[4] * motionData[320];
  xx[27] = xx[4] * motionData[321];
  xx[28] = xx[25];
  xx[29] = - xx[26];
  xx[30] = - xx[27];
  pm_math_Vector3_cross_ra(xx + 4, xx + 28, xx + 31);
  xx[28] = 0.27424;
  xx[29] = xx[5] * xx[28];
  xx[30] = 8.0e-3;
  xx[34] = xx[30] * xx[6];
  xx[35] = xx[29] - xx[34];
  xx[36] = xx[4] * xx[28];
  xx[37] = xx[4] * xx[30];
  xx[38] = xx[35];
  xx[39] = - xx[36];
  xx[40] = xx[37];
  pm_math_Vector3_cross_ra(xx + 4, xx + 38, xx + 41);
  xx[38] = motionData[316] * motionData[316];
  xx[39] = motionData[317] * motionData[318];
  xx[40] = motionData[315] * motionData[316];
  xx[44] = (xx[10] + xx[9]) * xx[8];
  xx[45] = xx[1] - (xx[7] + xx[38]) * xx[8];
  xx[46] = xx[8] * (xx[39] - xx[40]);
  pm_math_Vector3_cross_ra(xx + 44, xx + 20, xx + 47);
  pm_math_Quaternion_xform_ra(xx + 11, xx + 47, xx + 44);
  xx[7] = xx[5] * motionData[319];
  xx[9] = xx[4] * motionData[319];
  xx[10] = xx[24] + xx[9];
  xx[24] = xx[5] * motionData[321];
  xx[47] = - xx[7];
  xx[48] = xx[10];
  xx[49] = - xx[24];
  pm_math_Vector3_cross_ra(xx + 4, xx + 47, xx + 50);
  xx[47] = 0.513;
  xx[48] = xx[5] * xx[47];
  xx[49] = xx[4] * xx[47];
  xx[53] = xx[34] + xx[49];
  xx[34] = xx[5] * xx[30];
  xx[54] = xx[48];
  xx[55] = - xx[53];
  xx[56] = xx[34];
  pm_math_Vector3_cross_ra(xx + 4, xx + 54, xx + 57);
  xx[54] = xx[8] * (xx[16] - xx[15]);
  xx[55] = (xx[40] + xx[39]) * xx[8];
  xx[56] = xx[1] - (xx[38] + xx[2]) * xx[8];
  pm_math_Vector3_cross_ra(xx + 54, xx + 20, xx + 38);
  pm_math_Quaternion_xform_ra(xx + 11, xx + 38, xx + 54);
  xx[2] = xx[6] * motionData[319];
  xx[11] = xx[6] * motionData[320];
  xx[12] = xx[9] + xx[23];
  xx[13] = - xx[2];
  xx[14] = - xx[11];
  xx[15] = xx[12];
  pm_math_Vector3_cross_ra(xx + 4, xx + 13, xx + 38);
  xx[9] = xx[47] * xx[6];
  xx[13] = xx[28] * xx[6];
  xx[14] = xx[29] - xx[49];
  xx[60] = xx[9];
  xx[61] = - xx[13];
  xx[62] = xx[14];
  pm_math_Vector3_cross_ra(xx + 4, xx + 60, xx + 63);
  xx[66] = motionData[42];
  xx[67] = motionData[43];
  xx[68] = motionData[44];
  xx[69] = motionData[45];
  xx[4] = 0.9999999561635808;
  xx[5] = 2.960959925766019e-4;
  xx[6] = xx[4] * state[45] - xx[5] * state[46];
  xx[15] = xx[5] * state[45] + xx[4] * state[46];
  xx[16] = xx[4] * state[47] - xx[5] * state[48];
  xx[23] = xx[4] * state[48] + xx[5] * state[47];
  xx[70] = - xx[6];
  xx[71] = xx[15];
  xx[72] = xx[16];
  xx[73] = xx[23];
  pm_math_Quaternion_compose_ra(xx + 66, xx + 70, xx + 74);
  xx[78] = motionData[56];
  xx[79] = motionData[57];
  xx[80] = motionData[58];
  xx[81] = motionData[59];
  pm_math_Quaternion_compose_ra(xx + 74, xx + 78, xx + 82);
  xx[29] = xx[16] * xx[16];
  xx[49] = xx[23] * xx[23];
  xx[60] = (xx[29] + xx[49]) * xx[8] - xx[1];
  xx[61] = xx[6] * xx[23];
  xx[62] = xx[15] * xx[16];
  xx[74] = xx[61] + xx[62];
  xx[75] = xx[6] * xx[16];
  xx[76] = xx[15] * xx[23];
  xx[77] = xx[75] - xx[76];
  xx[86] = xx[60];
  xx[87] = - (xx[74] * xx[8]);
  xx[88] = xx[8] * xx[77];
  pm_math_Quaternion_inverseXform_ra(xx + 78, xx + 86, xx + 89);
  pm_math_Vector3_cross_ra(xx + 89, xx + 20, xx + 92);
  pm_math_Quaternion_xform_ra(xx + 82, xx + 92, xx + 89);
  xx[92] = motionData[60];
  xx[93] = motionData[61];
  xx[94] = motionData[62];
  pm_math_Vector3_cross_ra(xx + 86, xx + 92, xx + 95);
  pm_math_Quaternion_xform_ra(xx + 70, xx + 95, xx + 86);
  xx[95] = 0.283893415005525;
  xx[96] = 0.1300676268524588;
  xx[97] = 0.1419467075027625;
  xx[98] = 0.06503381342622938;
  xx[99] = - (xx[74] * xx[95] + xx[96] * xx[77]);
  xx[100] = - (xx[60] * xx[97]);
  xx[101] = xx[60] * xx[98];
  pm_math_Quaternion_xform_ra(xx + 70, xx + 99, xx + 102);
  xx[99] = xx[86] + xx[102];
  xx[100] = xx[87] + xx[103];
  xx[101] = xx[88] + xx[104];
  pm_math_Quaternion_xform_ra(xx + 66, xx + 99, xx + 86);
  xx[60] = xx[8] * (xx[61] - xx[62]);
  xx[61] = xx[15] * xx[15];
  xx[62] = (xx[49] + xx[61]) * xx[8] - xx[1];
  xx[49] = xx[6] * xx[15];
  xx[6] = xx[23] * xx[16];
  xx[15] = xx[49] + xx[6];
  xx[99] = xx[60];
  xx[100] = xx[62];
  xx[101] = - (xx[15] * xx[8]);
  pm_math_Quaternion_inverseXform_ra(xx + 78, xx + 99, xx + 102);
  pm_math_Vector3_cross_ra(xx + 102, xx + 20, xx + 105);
  pm_math_Quaternion_xform_ra(xx + 82, xx + 105, xx + 102);
  pm_math_Vector3_cross_ra(xx + 99, xx + 92, xx + 105);
  pm_math_Quaternion_xform_ra(xx + 70, xx + 105, xx + 99);
  xx[105] = xx[62] * xx[97] + xx[15] * xx[96];
  xx[106] = - (xx[97] * xx[60]);
  xx[107] = xx[98] * xx[60];
  pm_math_Quaternion_xform_ra(xx + 70, xx + 105, xx + 108);
  xx[105] = xx[99] + xx[108];
  xx[106] = xx[100] + xx[109];
  xx[107] = xx[101] + xx[110];
  pm_math_Quaternion_xform_ra(xx + 66, xx + 105, xx + 99);
  xx[15] = (xx[75] + xx[76]) * xx[8];
  xx[16] = xx[49] - xx[6];
  xx[6] = (xx[61] + xx[29]) * xx[8] - xx[1];
  xx[60] = - xx[15];
  xx[61] = xx[8] * xx[16];
  xx[62] = xx[6];
  pm_math_Quaternion_inverseXform_ra(xx + 78, xx + 60, xx + 74);
  pm_math_Vector3_cross_ra(xx + 74, xx + 20, xx + 77);
  pm_math_Quaternion_xform_ra(xx + 82, xx + 77, xx + 74);
  pm_math_Vector3_cross_ra(xx + 60, xx + 92, xx + 77);
  pm_math_Quaternion_xform_ra(xx + 70, xx + 77, xx + 60);
  xx[77] = xx[95] * xx[16] - xx[6] * xx[98];
  xx[78] = xx[97] * xx[15];
  xx[79] = - (xx[98] * xx[15]);
  pm_math_Quaternion_xform_ra(xx + 70, xx + 77, xx + 80);
  xx[70] = xx[60] + xx[80];
  xx[71] = xx[61] + xx[81];
  xx[72] = xx[62] + xx[82];
  pm_math_Quaternion_xform_ra(xx + 66, xx + 70, xx + 60);
  xx[66] = motionData[210];
  xx[67] = motionData[211];
  xx[68] = motionData[212];
  xx[69] = motionData[213];
  xx[6] = xx[4] * state[52] + xx[5] * state[53];
  xx[15] = xx[5] * state[52] - xx[4] * state[53];
  xx[16] = xx[5] * state[55] - xx[4] * state[54];
  xx[23] = xx[4] * state[55] + xx[5] * state[54];
  xx[4] = - xx[23];
  xx[70] = xx[6];
  xx[71] = xx[15];
  xx[72] = xx[16];
  xx[73] = xx[4];
  pm_math_Quaternion_compose_ra(xx + 66, xx + 70, xx + 77);
  xx[66] = (xx[16] * xx[16] + xx[23] * xx[23]) * xx[8] - xx[1];
  xx[67] = - ((xx[6] * xx[23] + xx[15] * xx[16]) * xx[8]);
  xx[68] = xx[8] * (xx[23] * xx[15] - xx[6] * xx[16]);
  pm_math_Vector3_cross_ra(xx + 66, xx + 20, xx + 69);
  pm_math_Quaternion_xform_ra(xx + 77, xx + 69, xx + 66);
  xx[69] = xx[15];
  xx[70] = xx[16];
  xx[71] = xx[4];
  xx[1] = 5.921919591936278e-4;
  xx[4] = 0.9999998246543264;
  xx[5] = xx[1] * xx[16] + xx[23] * xx[4];
  xx[29] = xx[1] * xx[15];
  xx[49] = xx[4] * xx[15];
  xx[81] = - xx[5];
  xx[82] = xx[29];
  xx[83] = - xx[49];
  pm_math_Vector3_cross_ra(xx + 69, xx + 81, xx + 92);
  xx[15] = xx[6] * xx[29];
  xx[72] = xx[6] * xx[49];
  xx[81] = xx[8] * (xx[92] + xx[6] * xx[5]);
  xx[82] = (xx[93] - xx[15]) * xx[8] - xx[4];
  xx[83] = xx[8] * (xx[94] + xx[72]) - xx[1];
  pm_math_Vector3_cross_ra(xx + 81, xx + 20, xx + 92);
  pm_math_Quaternion_xform_ra(xx + 77, xx + 92, xx + 81);
  xx[5] = xx[23] * xx[1] - xx[4] * xx[16];
  xx[92] = xx[5];
  xx[93] = xx[49];
  xx[94] = xx[29];
  pm_math_Vector3_cross_ra(xx + 69, xx + 92, xx + 95);
  xx[69] = xx[8] * (xx[95] - xx[6] * xx[5]);
  xx[70] = xx[1] + (xx[96] - xx[72]) * xx[8];
  xx[71] = xx[8] * (xx[97] - xx[15]) - xx[4];
  pm_math_Vector3_cross_ra(xx + 69, xx + 20, xx + 4);
  pm_math_Quaternion_xform_ra(xx + 77, xx + 4, xx + 20);
  J[17] = xx[17] + (xx[25] * xx[3] + xx[31]) * xx[8] + (xx[35] * xx[3] + xx[41])
    * xx[8];
  J[18] = xx[44] + motionData[321] + xx[8] * (xx[50] - xx[7] * xx[3]) + (xx[48] *
    xx[3] + xx[57]) * xx[8] - xx[30];
  J[19] = xx[54] + xx[8] * (xx[38] - xx[2] * xx[3]) + xx[8] * (xx[63] + xx[9] *
    xx[3]) - motionData[320] - xx[28];
  J[20] = xx[89] + xx[86];
  J[21] = xx[102] + xx[99];
  J[22] = xx[74] + xx[60];
  J[23] = xx[66];
  J[24] = xx[81];
  J[25] = xx[20];
  J[75] = xx[18] + xx[8] * (xx[32] - xx[26] * xx[3]) + xx[8] * (xx[42] - xx[36] *
    xx[3]) - motionData[321] + xx[30];
  J[76] = xx[45] + (xx[10] * xx[3] + xx[51]) * xx[8] + (xx[58] - xx[3] * xx[53])
    * xx[8];
  J[77] = xx[55] + motionData[319] + xx[8] * (xx[39] - xx[11] * xx[3]) + (xx[64]
    - xx[13] * xx[3]) * xx[8] - xx[47];
  J[78] = xx[90] + xx[87];
  J[79] = xx[103] + xx[100];
  J[80] = xx[75] + xx[61];
  J[81] = xx[67];
  J[82] = xx[82];
  J[83] = xx[21];
  J[133] = xx[19] + motionData[320] + xx[8] * (xx[33] - xx[27] * xx[3]) + (xx[37]
    * xx[3] + xx[43]) * xx[8] + xx[28];
  J[134] = xx[46] + xx[8] * (xx[52] - xx[24] * xx[3]) + xx[8] * (xx[59] + xx[34]
    * xx[3]) - motionData[319] + xx[47];
  J[135] = xx[56] + (xx[12] * xx[3] + xx[40]) * xx[8] + (xx[3] * xx[14] + xx[65])
    * xx[8];
  J[136] = xx[91] + xx[88];
  J[137] = xx[104] + xx[101];
  J[138] = xx[76] + xx[62];
  J[139] = xx[68];
  J[140] = xx[83];
  J[141] = xx[22];
  return 3;
}

static size_t computeAssemblyJacobian_10(const RuntimeDerivedValuesBundle *rtdv,
  const double *state, const int *modeVector, const double *motionData, double
  *J)
{
  const double *rtdvd = rtdv->mDoubles.mValues;
  const int *rtdvi = rtdv->mInts.mValues;
  double xx[49];
  (void) rtdvd;
  (void) rtdvi;
  (void) modeVector;
  (void) motionData;
  xx[0] = 0.0;
  xx[1] = 0.9998343934839863;
  xx[2] = 0.0181985056119827;
  xx[3] = xx[1] * state[68] - xx[2] * state[66];
  xx[4] = 0.2541;
  xx[5] = xx[3] * xx[4];
  xx[6] = 0.01433000000000001;
  xx[7] = xx[1] * state[69] + xx[2] * state[67];
  xx[8] = xx[6] * xx[7];
  xx[9] = xx[5] - xx[8];
  xx[10] = xx[1] * state[66] + xx[2] * state[68];
  xx[11] = xx[1] * state[67] - xx[2] * state[69];
  xx[12] = xx[11];
  xx[13] = xx[3];
  xx[14] = xx[7];
  xx[1] = xx[11] * xx[4];
  xx[2] = xx[11] * xx[6];
  xx[15] = xx[9];
  xx[16] = - xx[1];
  xx[17] = xx[2];
  pm_math_Vector3_cross_ra(xx + 12, xx + 15, xx + 18);
  xx[15] = 2.0;
  xx[16] = 0.284;
  xx[17] = xx[3] * xx[16];
  xx[21] = xx[8] - xx[17];
  xx[22] = xx[11] * xx[16];
  xx[23] = xx[21];
  xx[24] = xx[22];
  xx[25] = - xx[2];
  pm_math_Vector3_cross_ra(xx + 12, xx + 23, xx + 26);
  xx[23] = 8.82e-3;
  xx[24] = xx[3] * xx[23];
  xx[25] = xx[11] * xx[23];
  xx[29] = xx[8] + xx[25];
  xx[30] = xx[3] * xx[6];
  xx[31] = xx[24];
  xx[32] = - xx[29];
  xx[33] = xx[30];
  pm_math_Vector3_cross_ra(xx + 12, xx + 31, xx + 34);
  xx[6] = 0.5109;
  xx[31] = xx[3] * xx[6];
  xx[3] = xx[11] * xx[6];
  xx[11] = xx[8] - xx[3];
  xx[37] = xx[31];
  xx[38] = xx[11];
  xx[39] = - xx[30];
  pm_math_Vector3_cross_ra(xx + 12, xx + 37, xx + 40);
  xx[8] = xx[23] * xx[7];
  xx[23] = xx[4] * xx[7];
  xx[4] = xx[5] - xx[25];
  xx[37] = xx[8];
  xx[38] = - xx[23];
  xx[39] = xx[4];
  pm_math_Vector3_cross_ra(xx + 12, xx + 37, xx + 43);
  xx[5] = xx[6] * xx[7];
  xx[6] = xx[16] * xx[7];
  xx[7] = xx[3] + xx[17];
  xx[37] = xx[5];
  xx[38] = xx[6];
  xx[39] = - xx[7];
  pm_math_Vector3_cross_ra(xx + 12, xx + 37, xx + 46);
  xx[3] = 0.02989999999999998;
  xx[12] = 0.5197200000000001;
  xx[13] = xx[2] * xx[10];
  xx[2] = xx[30] * xx[10];
  J[29] = (xx[9] * xx[10] + xx[18]) * xx[15] + (xx[10] * xx[21] + xx[26]) * xx
    [15];
  J[30] = (xx[24] * xx[10] + xx[34]) * xx[15] + (xx[31] * xx[10] + xx[40]) * xx
    [15];
  J[31] = xx[15] * (xx[43] + xx[8] * xx[10]) + xx[15] * (xx[46] + xx[5] * xx[10])
    + xx[3];
  J[87] = xx[15] * (xx[19] - xx[1] * xx[10]) + xx[15] * (xx[27] + xx[22] * xx[10]);
  J[88] = (xx[35] - xx[10] * xx[29]) * xx[15] + (xx[11] * xx[10] + xx[41]) * xx
    [15];
  J[89] = (xx[44] - xx[23] * xx[10]) * xx[15] + (xx[6] * xx[10] + xx[47]) * xx
    [15] - xx[12];
  J[145] = (xx[13] + xx[20]) * xx[15] + (xx[28] - xx[13]) * xx[15] - xx[3];
  J[146] = xx[15] * (xx[36] + xx[2]) + xx[15] * (xx[42] - xx[2]) + xx[12];
  J[147] = (xx[10] * xx[4] + xx[45]) * xx[15] + (xx[48] - xx[10] * xx[7]) * xx
    [15];
  return 3;
}

static size_t computeAssemblyJacobian_11(const RuntimeDerivedValuesBundle *rtdv,
  const double *state, const int *modeVector, const double *motionData, double
  *J)
{
  const double *rtdvd = rtdv->mDoubles.mValues;
  const int *rtdvi = rtdv->mInts.mValues;
  double xx[109];
  (void) rtdvd;
  (void) rtdvi;
  (void) modeVector;
  xx[0] = 0.0;
  xx[1] = 0.9998343934839863;
  xx[2] = 0.0181985056119827;
  xx[3] = xx[1] * state[98] + xx[2] * state[100];
  xx[4] = xx[1] * state[99] - xx[2] * state[101];
  xx[5] = xx[1] * state[100] - xx[2] * state[98];
  xx[6] = xx[1] * state[101] + xx[2] * state[99];
  xx[7] = motionData[329];
  xx[8] = motionData[330];
  xx[9] = motionData[331];
  xx[10] = motionData[332];
  pm_math_Quaternion_compose_ra(xx + 3, xx + 7, xx + 11);
  xx[1] = 1.0;
  xx[2] = motionData[331] * motionData[331];
  xx[7] = motionData[332] * motionData[332];
  xx[8] = 2.0;
  xx[9] = motionData[330] * motionData[331];
  xx[10] = motionData[329] * motionData[332];
  xx[15] = motionData[329] * motionData[331];
  xx[16] = motionData[330] * motionData[332];
  xx[17] = xx[1] - (xx[2] + xx[7]) * xx[8];
  xx[18] = xx[8] * (xx[9] - xx[10]);
  xx[19] = (xx[15] + xx[16]) * xx[8];
  xx[20] = 0.1378799999999999;
  xx[21] = 0.29899;
  xx[22] = 6.500000000000001e-3;
  xx[23] = xx[20];
  xx[24] = xx[21];
  xx[25] = - xx[22];
  pm_math_Vector3_cross_ra(xx + 17, xx + 23, xx + 26);
  pm_math_Quaternion_xform_ra(xx + 11, xx + 26, xx + 17);
  xx[26] = xx[5] * motionData[334];
  xx[27] = xx[6] * motionData[335];
  xx[28] = xx[26] + xx[27];
  xx[29] = xx[4] * motionData[334];
  xx[30] = xx[4] * motionData[335];
  xx[31] = xx[28];
  xx[32] = - xx[29];
  xx[33] = - xx[30];
  pm_math_Vector3_cross_ra(xx + 4, xx + 31, xx + 34);
  xx[31] = 0.05344999999999999;
  xx[32] = xx[31] * xx[6];
  xx[33] = 0.26683;
  xx[37] = xx[5] * xx[33];
  xx[38] = xx[32] - xx[37];
  xx[39] = xx[4] * xx[33];
  xx[40] = xx[4] * xx[31];
  xx[41] = xx[38];
  xx[42] = xx[39];
  xx[43] = - xx[40];
  pm_math_Vector3_cross_ra(xx + 4, xx + 41, xx + 44);
  xx[41] = motionData[330] * motionData[330];
  xx[42] = motionData[331] * motionData[332];
  xx[43] = motionData[329] * motionData[330];
  xx[47] = (xx[10] + xx[9]) * xx[8];
  xx[48] = xx[1] - (xx[7] + xx[41]) * xx[8];
  xx[49] = xx[8] * (xx[42] - xx[43]);
  pm_math_Vector3_cross_ra(xx + 47, xx + 23, xx + 50);
  pm_math_Quaternion_xform_ra(xx + 11, xx + 50, xx + 47);
  xx[7] = xx[5] * motionData[333];
  xx[9] = xx[4] * motionData[333];
  xx[10] = xx[27] + xx[9];
  xx[27] = xx[5] * motionData[335];
  xx[50] = - xx[7];
  xx[51] = xx[10];
  xx[52] = - xx[27];
  pm_math_Vector3_cross_ra(xx + 4, xx + 50, xx + 53);
  xx[50] = 0.14279;
  xx[51] = xx[5] * xx[50];
  xx[52] = xx[4] * xx[50];
  xx[56] = xx[32] - xx[52];
  xx[32] = xx[5] * xx[31];
  xx[57] = xx[51];
  xx[58] = xx[56];
  xx[59] = - xx[32];
  pm_math_Vector3_cross_ra(xx + 4, xx + 57, xx + 60);
  xx[57] = xx[8] * (xx[16] - xx[15]);
  xx[58] = (xx[43] + xx[42]) * xx[8];
  xx[59] = xx[1] - (xx[41] + xx[2]) * xx[8];
  pm_math_Vector3_cross_ra(xx + 57, xx + 23, xx + 41);
  pm_math_Quaternion_xform_ra(xx + 11, xx + 41, xx + 57);
  xx[1] = xx[6] * motionData[333];
  xx[2] = xx[6] * motionData[334];
  xx[11] = xx[9] + xx[26];
  xx[12] = - xx[1];
  xx[13] = - xx[2];
  xx[14] = xx[11];
  pm_math_Vector3_cross_ra(xx + 4, xx + 12, xx + 41);
  xx[9] = xx[50] * xx[6];
  xx[12] = xx[33] * xx[6];
  xx[13] = xx[52] + xx[37];
  xx[14] = xx[9];
  xx[15] = xx[12];
  xx[16] = - xx[13];
  pm_math_Vector3_cross_ra(xx + 4, xx + 14, xx + 63);
  xx[66] = motionData[112];
  xx[67] = motionData[113];
  xx[68] = motionData[114];
  xx[69] = motionData[115];
  xx[70] = state[112];
  xx[71] = state[113];
  xx[72] = state[114];
  xx[73] = state[115];
  xx[4] = 0.9999188709407687;
  xx[5] = 1.729133611397341e-4;
  xx[6] = 0.01273662563857442;
  xx[14] = 2.20251143642801e-6;
  xx[74] = xx[4];
  xx[75] = - xx[5];
  xx[76] = xx[6];
  xx[77] = - xx[14];
  pm_math_Quaternion_composeInverse_ra(xx + 70, xx + 74, xx + 78);
  pm_math_Quaternion_compose_ra(xx + 66, xx + 78, xx + 70);
  xx[74] = motionData[133];
  xx[75] = motionData[134];
  xx[76] = motionData[135];
  xx[77] = motionData[136];
  pm_math_Quaternion_compose_ra(xx + 70, xx + 74, xx + 82);
  xx[70] = 0.9996755567249835;
  xx[71] = - 8.809330994983285e-6;
  xx[72] = - 0.02547118389454986;
  pm_math_Quaternion_inverseXform_ra(xx + 74, xx + 70, xx + 86);
  pm_math_Vector3_cross_ra(xx + 86, xx + 23, xx + 89);
  pm_math_Quaternion_xform_ra(xx + 82, xx + 89, xx + 86);
  xx[89] = motionData[137];
  xx[90] = motionData[138];
  xx[91] = motionData[139];
  pm_math_Vector3_cross_ra(xx + 70, xx + 89, xx + 92);
  pm_math_Quaternion_xform_ra(xx + 78, xx + 92, xx + 70);
  xx[92] = - 1.565104140975318e-3;
  xx[93] = 0.01752317107628744;
  xx[94] = - 0.06143219441052305;
  pm_math_Quaternion_xform_ra(xx + 78, xx + 92, xx + 95);
  xx[92] = xx[70] + xx[95];
  xx[93] = xx[71] + xx[96];
  xx[94] = xx[72] + xx[97];
  pm_math_Quaternion_xform_ra(xx + 66, xx + 92, xx + 70);
  xx[92] = motionData[134];
  xx[93] = motionData[135];
  xx[94] = motionData[136];
  xx[15] = 3.458547708100935e-4;
  xx[16] = 0.9999999401922369;
  xx[26] = xx[15] * motionData[135] + xx[16] * motionData[136];
  xx[37] = xx[15] * motionData[134];
  xx[95] = - xx[26];
  xx[96] = xx[37];
  xx[97] = xx[16] * motionData[134];
  pm_math_Vector3_cross_ra(xx + 92, xx + 95, xx + 98);
  xx[92] = xx[8] * (xx[98] + xx[26] * motionData[133]);
  xx[93] = xx[16] + (xx[99] - xx[37] * motionData[133]) * xx[8];
  xx[94] = xx[8] * (xx[100] - xx[16] * motionData[133] * motionData[134]) - xx
    [15];
  pm_math_Vector3_cross_ra(xx + 92, xx + 23, xx + 95);
  pm_math_Quaternion_xform_ra(xx + 82, xx + 95, xx + 92);
  xx[95] = xx[16] * motionData[139] + xx[15] * motionData[138];
  xx[96] = - (xx[15] * motionData[137]);
  xx[97] = - (xx[16] * motionData[137]);
  pm_math_Quaternion_xform_ra(xx + 78, xx + 95, xx + 98);
  xx[95] = - 0.0177372062878282;
  xx[96] = - 2.539611720865432e-6;
  xx[97] = - 7.342999962176374e-3;
  pm_math_Quaternion_xform_ra(xx + 78, xx + 95, xx + 101);
  xx[95] = xx[98] + xx[101];
  xx[96] = xx[99] + xx[102];
  xx[97] = xx[100] + xx[103];
  pm_math_Quaternion_xform_ra(xx + 66, xx + 95, xx + 98);
  xx[95] = 0.02547118541792448;
  xx[96] = 3.457425605555716e-4;
  xx[97] = 0.9996754969366247;
  pm_math_Quaternion_inverseXform_ra(xx + 74, xx + 95, xx + 101);
  pm_math_Vector3_cross_ra(xx + 101, xx + 23, xx + 73);
  pm_math_Quaternion_xform_ra(xx + 82, xx + 73, xx + 23);
  pm_math_Vector3_cross_ra(xx + 95, xx + 89, xx + 73);
  pm_math_Quaternion_xform_ra(xx + 78, xx + 73, xx + 82);
  xx[73] = 0.06142613026408447;
  xx[74] = 7.791863919619067e-3;
  xx[75] = - 1.567799088051977e-3;
  pm_math_Quaternion_xform_ra(xx + 78, xx + 73, xx + 89);
  xx[73] = xx[82] + xx[89];
  xx[74] = xx[83] + xx[90];
  xx[75] = xx[84] + xx[91];
  pm_math_Quaternion_xform_ra(xx + 66, xx + 73, xx + 76);
  xx[66] = motionData[245];
  xx[67] = motionData[246];
  xx[68] = motionData[247];
  xx[69] = motionData[248];
  xx[79] = - xx[4];
  xx[80] = xx[5];
  xx[81] = - xx[6];
  xx[82] = xx[14];
  xx[101] = - state[119];
  xx[102] = - state[120];
  xx[103] = - state[121];
  xx[104] = - state[122];
  pm_math_Quaternion_compose_ra(xx + 79, xx + 101, xx + 105);
  pm_math_Quaternion_compose_ra(xx + 66, xx + 105, xx + 79);
  xx[4] = xx[21] * xx[81];
  xx[5] = xx[22] * xx[82];
  xx[6] = xx[4] - xx[5];
  xx[14] = xx[21] * xx[80];
  xx[66] = xx[6];
  xx[67] = - xx[14];
  xx[68] = xx[22] * xx[80];
  pm_math_Vector3_cross_ra(xx + 80, xx + 66, xx + 73);
  xx[15] = xx[20] * xx[80];
  xx[16] = xx[15] - xx[5];
  xx[5] = xx[22] * xx[81];
  xx[66] = - (xx[20] * xx[81]);
  xx[67] = xx[16];
  xx[68] = xx[5];
  pm_math_Vector3_cross_ra(xx + 80, xx + 66, xx + 83);
  xx[26] = xx[20] * xx[82];
  xx[37] = xx[15] + xx[4];
  xx[66] = - xx[26];
  xx[67] = - (xx[21] * xx[82]);
  xx[68] = xx[37];
  pm_math_Vector3_cross_ra(xx + 80, xx + 66, xx + 89);
  J[43] = xx[17] + (xx[28] * xx[3] + xx[34]) * xx[8] + (xx[3] * xx[38] + xx[44])
    * xx[8];
  J[44] = xx[47] + motionData[335] + xx[8] * (xx[53] - xx[7] * xx[3]) + (xx[51] *
    xx[3] + xx[60]) * xx[8] + xx[31];
  J[45] = xx[57] + xx[8] * (xx[41] - xx[1] * xx[3]) + xx[8] * (xx[63] + xx[9] *
    xx[3]) - motionData[334] + xx[33];
  J[49] = xx[86] + xx[70];
  J[50] = xx[92] + xx[98];
  J[51] = xx[23] + xx[76];
  J[52] = (xx[79] * xx[6] + xx[73]) * xx[8];
  J[53] = (xx[83] - xx[20] * xx[79] * xx[81]) * xx[8] - xx[22];
  J[54] = xx[8] * (xx[89] - xx[79] * xx[26]) - xx[21];
  J[101] = xx[18] + xx[8] * (xx[35] - xx[29] * xx[3]) + xx[8] * (xx[45] + xx[39]
    * xx[3]) - motionData[335] - xx[31];
  J[102] = xx[48] + (xx[10] * xx[3] + xx[54]) * xx[8] + (xx[56] * xx[3] + xx[61])
    * xx[8];
  J[103] = xx[58] + motionData[333] + xx[8] * (xx[42] - xx[2] * xx[3]) + (xx[12]
    * xx[3] + xx[64]) * xx[8] - xx[50];
  J[107] = xx[87] + xx[71];
  J[108] = xx[93] + xx[99];
  J[109] = xx[24] + xx[77];
  J[110] = xx[22] + xx[8] * (xx[74] - xx[79] * xx[14]);
  J[111] = (xx[79] * xx[16] + xx[84]) * xx[8];
  J[112] = xx[20] + (xx[90] - xx[21] * xx[79] * xx[82]) * xx[8];
  J[159] = xx[19] + motionData[334] + xx[8] * (xx[36] - xx[30] * xx[3]) + (xx[46]
    - xx[40] * xx[3]) * xx[8] - xx[33];
  J[160] = xx[49] + xx[8] * (xx[55] - xx[27] * xx[3]) + xx[8] * (xx[62] - xx[32]
    * xx[3]) - motionData[333] + xx[50];
  J[161] = xx[59] + (xx[11] * xx[3] + xx[43]) * xx[8] + (xx[65] - xx[3] * xx[13])
    * xx[8];
  J[165] = xx[88] + xx[72];
  J[166] = xx[94] + xx[100];
  J[167] = xx[25] + xx[78];
  J[168] = xx[21] + (xx[22] * xx[79] * xx[80] + xx[75]) * xx[8];
  J[169] = xx[8] * (xx[85] + xx[79] * xx[5]) - xx[20];
  J[170] = (xx[79] * xx[37] + xx[91]) * xx[8];
  return 3;
}

static size_t computeAssemblyJacobian_12(const RuntimeDerivedValuesBundle *rtdv,
  const double *state, const int *modeVector, const double *motionData, double
  *J)
{
  const double *rtdvd = rtdv->mDoubles.mValues;
  const int *rtdvi = rtdv->mInts.mValues;
  double xx[109];
  (void) rtdvd;
  (void) rtdvi;
  (void) modeVector;
  xx[0] = 0.0;
  xx[1] = 0.9998343934839863;
  xx[2] = 0.0181985056119827;
  xx[3] = xx[1] * state[98] + xx[2] * state[100];
  xx[4] = xx[1] * state[99] - xx[2] * state[101];
  xx[5] = xx[1] * state[100] - xx[2] * state[98];
  xx[6] = xx[1] * state[101] + xx[2] * state[99];
  xx[7] = motionData[329];
  xx[8] = motionData[330];
  xx[9] = motionData[331];
  xx[10] = motionData[332];
  pm_math_Quaternion_compose_ra(xx + 3, xx + 7, xx + 11);
  xx[1] = 1.0;
  xx[2] = motionData[331] * motionData[331];
  xx[7] = motionData[332] * motionData[332];
  xx[8] = 2.0;
  xx[9] = motionData[330] * motionData[331];
  xx[10] = motionData[329] * motionData[332];
  xx[15] = motionData[329] * motionData[331];
  xx[16] = motionData[330] * motionData[332];
  xx[17] = xx[1] - (xx[2] + xx[7]) * xx[8];
  xx[18] = xx[8] * (xx[9] - xx[10]);
  xx[19] = (xx[15] + xx[16]) * xx[8];
  xx[20] = 0.15472;
  xx[21] = 0.29499;
  xx[22] = 2.200000000000003e-3;
  xx[23] = - xx[20];
  xx[24] = xx[21];
  xx[25] = - xx[22];
  pm_math_Vector3_cross_ra(xx + 17, xx + 23, xx + 26);
  pm_math_Quaternion_xform_ra(xx + 11, xx + 26, xx + 17);
  xx[26] = xx[5] * motionData[334];
  xx[27] = xx[6] * motionData[335];
  xx[28] = xx[26] + xx[27];
  xx[29] = xx[4] * motionData[334];
  xx[30] = xx[4] * motionData[335];
  xx[31] = xx[28];
  xx[32] = - xx[29];
  xx[33] = - xx[30];
  pm_math_Vector3_cross_ra(xx + 4, xx + 31, xx + 34);
  xx[31] = 0.05344999999999999;
  xx[32] = xx[31] * xx[6];
  xx[33] = 0.26683;
  xx[37] = xx[5] * xx[33];
  xx[38] = xx[32] - xx[37];
  xx[39] = xx[4] * xx[33];
  xx[40] = xx[4] * xx[31];
  xx[41] = xx[38];
  xx[42] = xx[39];
  xx[43] = - xx[40];
  pm_math_Vector3_cross_ra(xx + 4, xx + 41, xx + 44);
  xx[41] = motionData[330] * motionData[330];
  xx[42] = motionData[331] * motionData[332];
  xx[43] = motionData[329] * motionData[330];
  xx[47] = (xx[10] + xx[9]) * xx[8];
  xx[48] = xx[1] - (xx[7] + xx[41]) * xx[8];
  xx[49] = xx[8] * (xx[42] - xx[43]);
  pm_math_Vector3_cross_ra(xx + 47, xx + 23, xx + 50);
  pm_math_Quaternion_xform_ra(xx + 11, xx + 50, xx + 47);
  xx[7] = xx[5] * motionData[333];
  xx[9] = xx[4] * motionData[333];
  xx[10] = xx[27] + xx[9];
  xx[27] = xx[5] * motionData[335];
  xx[50] = - xx[7];
  xx[51] = xx[10];
  xx[52] = - xx[27];
  pm_math_Vector3_cross_ra(xx + 4, xx + 50, xx + 53);
  xx[50] = 0.14279;
  xx[51] = xx[5] * xx[50];
  xx[52] = xx[4] * xx[50];
  xx[56] = xx[32] - xx[52];
  xx[32] = xx[5] * xx[31];
  xx[57] = xx[51];
  xx[58] = xx[56];
  xx[59] = - xx[32];
  pm_math_Vector3_cross_ra(xx + 4, xx + 57, xx + 60);
  xx[57] = xx[8] * (xx[16] - xx[15]);
  xx[58] = (xx[43] + xx[42]) * xx[8];
  xx[59] = xx[1] - (xx[41] + xx[2]) * xx[8];
  pm_math_Vector3_cross_ra(xx + 57, xx + 23, xx + 41);
  pm_math_Quaternion_xform_ra(xx + 11, xx + 41, xx + 57);
  xx[1] = xx[6] * motionData[333];
  xx[2] = xx[6] * motionData[334];
  xx[11] = xx[9] + xx[26];
  xx[12] = - xx[1];
  xx[13] = - xx[2];
  xx[14] = xx[11];
  pm_math_Vector3_cross_ra(xx + 4, xx + 12, xx + 41);
  xx[9] = xx[50] * xx[6];
  xx[12] = xx[33] * xx[6];
  xx[13] = xx[52] + xx[37];
  xx[14] = xx[9];
  xx[15] = xx[12];
  xx[16] = - xx[13];
  pm_math_Vector3_cross_ra(xx + 4, xx + 14, xx + 63);
  xx[66] = motionData[112];
  xx[67] = motionData[113];
  xx[68] = motionData[114];
  xx[69] = motionData[115];
  xx[70] = state[112];
  xx[71] = state[113];
  xx[72] = state[114];
  xx[73] = state[115];
  xx[4] = 0.9999188709407687;
  xx[5] = 1.729133611397341e-4;
  xx[6] = 0.01273662563857442;
  xx[14] = 2.20251143642801e-6;
  xx[74] = xx[4];
  xx[75] = - xx[5];
  xx[76] = xx[6];
  xx[77] = - xx[14];
  pm_math_Quaternion_composeInverse_ra(xx + 70, xx + 74, xx + 78);
  pm_math_Quaternion_compose_ra(xx + 66, xx + 78, xx + 70);
  xx[74] = motionData[133];
  xx[75] = motionData[134];
  xx[76] = motionData[135];
  xx[77] = motionData[136];
  pm_math_Quaternion_compose_ra(xx + 70, xx + 74, xx + 82);
  xx[70] = 0.9996755567249835;
  xx[71] = - 8.809330994983285e-6;
  xx[72] = - 0.02547118389454986;
  pm_math_Quaternion_inverseXform_ra(xx + 74, xx + 70, xx + 86);
  pm_math_Vector3_cross_ra(xx + 86, xx + 23, xx + 89);
  pm_math_Quaternion_xform_ra(xx + 82, xx + 89, xx + 86);
  xx[89] = motionData[137];
  xx[90] = motionData[138];
  xx[91] = motionData[139];
  pm_math_Vector3_cross_ra(xx + 70, xx + 89, xx + 92);
  pm_math_Quaternion_xform_ra(xx + 78, xx + 92, xx + 70);
  xx[92] = - 1.565104140975318e-3;
  xx[93] = 0.01752317107628744;
  xx[94] = - 0.06143219441052305;
  pm_math_Quaternion_xform_ra(xx + 78, xx + 92, xx + 95);
  xx[92] = xx[70] + xx[95];
  xx[93] = xx[71] + xx[96];
  xx[94] = xx[72] + xx[97];
  pm_math_Quaternion_xform_ra(xx + 66, xx + 92, xx + 70);
  xx[92] = motionData[134];
  xx[93] = motionData[135];
  xx[94] = motionData[136];
  xx[15] = 3.458547708100935e-4;
  xx[16] = 0.9999999401922369;
  xx[26] = xx[15] * motionData[135] + xx[16] * motionData[136];
  xx[37] = xx[15] * motionData[134];
  xx[95] = - xx[26];
  xx[96] = xx[37];
  xx[97] = xx[16] * motionData[134];
  pm_math_Vector3_cross_ra(xx + 92, xx + 95, xx + 98);
  xx[92] = xx[8] * (xx[98] + xx[26] * motionData[133]);
  xx[93] = xx[16] + (xx[99] - xx[37] * motionData[133]) * xx[8];
  xx[94] = xx[8] * (xx[100] - xx[16] * motionData[133] * motionData[134]) - xx
    [15];
  pm_math_Vector3_cross_ra(xx + 92, xx + 23, xx + 95);
  pm_math_Quaternion_xform_ra(xx + 82, xx + 95, xx + 92);
  xx[95] = xx[16] * motionData[139] + xx[15] * motionData[138];
  xx[96] = - (xx[15] * motionData[137]);
  xx[97] = - (xx[16] * motionData[137]);
  pm_math_Quaternion_xform_ra(xx + 78, xx + 95, xx + 98);
  xx[95] = - 0.0177372062878282;
  xx[96] = - 2.539611720865432e-6;
  xx[97] = - 7.342999962176374e-3;
  pm_math_Quaternion_xform_ra(xx + 78, xx + 95, xx + 101);
  xx[95] = xx[98] + xx[101];
  xx[96] = xx[99] + xx[102];
  xx[97] = xx[100] + xx[103];
  pm_math_Quaternion_xform_ra(xx + 66, xx + 95, xx + 98);
  xx[95] = 0.02547118541792448;
  xx[96] = 3.457425605555716e-4;
  xx[97] = 0.9996754969366247;
  pm_math_Quaternion_inverseXform_ra(xx + 74, xx + 95, xx + 101);
  pm_math_Vector3_cross_ra(xx + 101, xx + 23, xx + 73);
  pm_math_Quaternion_xform_ra(xx + 82, xx + 73, xx + 23);
  pm_math_Vector3_cross_ra(xx + 95, xx + 89, xx + 73);
  pm_math_Quaternion_xform_ra(xx + 78, xx + 73, xx + 82);
  xx[73] = 0.06142613026408447;
  xx[74] = 7.791863919619067e-3;
  xx[75] = - 1.567799088051977e-3;
  pm_math_Quaternion_xform_ra(xx + 78, xx + 73, xx + 89);
  xx[73] = xx[82] + xx[89];
  xx[74] = xx[83] + xx[90];
  xx[75] = xx[84] + xx[91];
  pm_math_Quaternion_xform_ra(xx + 66, xx + 73, xx + 76);
  xx[66] = motionData[245];
  xx[67] = motionData[246];
  xx[68] = motionData[247];
  xx[69] = motionData[248];
  xx[79] = - xx[4];
  xx[80] = xx[5];
  xx[81] = - xx[6];
  xx[82] = xx[14];
  xx[101] = - state[119];
  xx[102] = - state[120];
  xx[103] = - state[121];
  xx[104] = - state[122];
  pm_math_Quaternion_compose_ra(xx + 79, xx + 101, xx + 105);
  pm_math_Quaternion_compose_ra(xx + 66, xx + 105, xx + 79);
  xx[4] = xx[21] * xx[81];
  xx[5] = xx[22] * xx[82];
  xx[6] = xx[4] - xx[5];
  xx[14] = xx[21] * xx[80];
  xx[66] = xx[6];
  xx[67] = - xx[14];
  xx[68] = xx[22] * xx[80];
  pm_math_Vector3_cross_ra(xx + 80, xx + 66, xx + 73);
  xx[15] = xx[20] * xx[80];
  xx[16] = xx[5] + xx[15];
  xx[5] = xx[22] * xx[81];
  xx[66] = xx[20] * xx[81];
  xx[67] = - xx[16];
  xx[68] = xx[5];
  pm_math_Vector3_cross_ra(xx + 80, xx + 66, xx + 83);
  xx[26] = xx[20] * xx[82];
  xx[37] = xx[4] - xx[15];
  xx[66] = xx[26];
  xx[67] = - (xx[21] * xx[82]);
  xx[68] = xx[37];
  pm_math_Vector3_cross_ra(xx + 80, xx + 66, xx + 89);
  J[43] = xx[17] + (xx[28] * xx[3] + xx[34]) * xx[8] + (xx[3] * xx[38] + xx[44])
    * xx[8];
  J[44] = xx[47] + motionData[335] + xx[8] * (xx[53] - xx[7] * xx[3]) + (xx[51] *
    xx[3] + xx[60]) * xx[8] + xx[31];
  J[45] = xx[57] + xx[8] * (xx[41] - xx[1] * xx[3]) + xx[8] * (xx[63] + xx[9] *
    xx[3]) - motionData[334] + xx[33];
  J[49] = xx[86] + xx[70];
  J[50] = xx[92] + xx[98];
  J[51] = xx[23] + xx[76];
  J[52] = (xx[79] * xx[6] + xx[73]) * xx[8];
  J[53] = (xx[20] * xx[79] * xx[81] + xx[83]) * xx[8] - xx[22];
  J[54] = xx[8] * (xx[89] + xx[79] * xx[26]) - xx[21];
  J[101] = xx[18] + xx[8] * (xx[35] - xx[29] * xx[3]) + xx[8] * (xx[45] + xx[39]
    * xx[3]) - motionData[335] - xx[31];
  J[102] = xx[48] + (xx[10] * xx[3] + xx[54]) * xx[8] + (xx[56] * xx[3] + xx[61])
    * xx[8];
  J[103] = xx[58] + motionData[333] + xx[8] * (xx[42] - xx[2] * xx[3]) + (xx[12]
    * xx[3] + xx[64]) * xx[8] - xx[50];
  J[107] = xx[87] + xx[71];
  J[108] = xx[93] + xx[99];
  J[109] = xx[24] + xx[77];
  J[110] = xx[22] + xx[8] * (xx[74] - xx[79] * xx[14]);
  J[111] = (xx[84] - xx[79] * xx[16]) * xx[8];
  J[112] = (xx[90] - xx[21] * xx[79] * xx[82]) * xx[8] - xx[20];
  J[159] = xx[19] + motionData[334] + xx[8] * (xx[36] - xx[30] * xx[3]) + (xx[46]
    - xx[40] * xx[3]) * xx[8] - xx[33];
  J[160] = xx[49] + xx[8] * (xx[55] - xx[27] * xx[3]) + xx[8] * (xx[62] - xx[32]
    * xx[3]) - motionData[333] + xx[50];
  J[161] = xx[59] + (xx[11] * xx[3] + xx[43]) * xx[8] + (xx[65] - xx[3] * xx[13])
    * xx[8];
  J[165] = xx[88] + xx[72];
  J[166] = xx[94] + xx[100];
  J[167] = xx[25] + xx[78];
  J[168] = xx[21] + (xx[22] * xx[79] * xx[80] + xx[75]) * xx[8];
  J[169] = xx[20] + xx[8] * (xx[85] + xx[79] * xx[5]);
  J[170] = (xx[79] * xx[37] + xx[91]) * xx[8];
  return 3;
}

static size_t computeAssemblyJacobian_13(const RuntimeDerivedValuesBundle *rtdv,
  const double *state, const int *modeVector, const double *motionData, double
  *J)
{
  const double *rtdvd = rtdv->mDoubles.mValues;
  const int *rtdvi = rtdv->mInts.mValues;
  double xx[90];
  (void) rtdvd;
  (void) rtdvi;
  (void) modeVector;
  xx[0] = 0.0;
  xx[1] = 0.9999188709407687;
  xx[2] = 1.729133611399561e-4;
  xx[3] = 0.01273662563857442;
  xx[4] = 2.202511436425175e-6;
  xx[5] = - xx[1];
  xx[6] = - xx[2];
  xx[7] = - xx[3];
  xx[8] = - xx[4];
  xx[9] = - state[13];
  xx[10] = - state[14];
  xx[11] = - state[15];
  xx[12] = - state[16];
  pm_math_Quaternion_compose_ra(xx + 5, xx + 9, xx + 13);
  xx[5] = 0.29899;
  xx[6] = xx[5] * xx[15];
  xx[7] = 6.500000000000001e-3;
  xx[8] = xx[7] * xx[16];
  xx[9] = xx[6] + xx[8];
  xx[10] = xx[5] * xx[14];
  xx[17] = - xx[9];
  xx[18] = xx[10];
  xx[19] = xx[7] * xx[14];
  pm_math_Vector3_cross_ra(xx + 14, xx + 17, xx + 20);
  xx[11] = 2.0;
  xx[12] = 0.1378799999999999;
  xx[17] = xx[12] * xx[14];
  xx[18] = xx[17] - xx[8];
  xx[8] = xx[7] * xx[15];
  xx[23] = - (xx[12] * xx[15]);
  xx[24] = xx[18];
  xx[25] = xx[8];
  pm_math_Vector3_cross_ra(xx + 14, xx + 23, xx + 26);
  xx[19] = xx[12] * xx[16];
  xx[23] = xx[17] - xx[6];
  xx[29] = - xx[19];
  xx[30] = xx[5] * xx[16];
  xx[31] = xx[23];
  pm_math_Vector3_cross_ra(xx + 14, xx + 29, xx + 32);
  xx[35] = state[20];
  xx[36] = - state[21];
  xx[37] = - state[22];
  xx[38] = - state[23];
  pm_math_Quaternion_compose_ra(xx + 1, xx + 35, xx + 39);
  xx[1] = motionData[21];
  xx[2] = motionData[22];
  xx[3] = motionData[23];
  xx[4] = motionData[24];
  pm_math_Quaternion_compose_ra(xx + 39, xx + 1, xx + 35);
  xx[29] = - 0.9996755567249835;
  xx[30] = - 8.80933099498327e-6;
  xx[31] = 0.02547118389454986;
  pm_math_Quaternion_inverseXform_ra(xx + 39, xx + 29, xx + 43);
  pm_math_Quaternion_inverseXform_ra(xx + 1, xx + 43, xx + 29);
  xx[46] = 0.8225246892800159;
  xx[47] = 0.264;
  xx[48] = - 0.07704855223048318;
  pm_math_Vector3_cross_ra(xx + 29, xx + 46, xx + 49);
  pm_math_Quaternion_xform_ra(xx + 35, xx + 49, xx + 29);
  xx[49] = motionData[25];
  xx[50] = motionData[26];
  xx[51] = motionData[27];
  pm_math_Vector3_cross_ra(xx + 43, xx + 49, xx + 52);
  pm_math_Quaternion_xform_ra(xx + 39, xx + 52, xx + 43);
  xx[6] = 0.9999999401922369;
  xx[17] = 3.458547708105375e-4;
  xx[24] = xx[6] * xx[42] - xx[17] * xx[41];
  xx[25] = xx[17] * xx[40];
  xx[52] = xx[24];
  xx[53] = xx[25];
  xx[54] = - (xx[6] * xx[40]);
  pm_math_Vector3_cross_ra(xx + 40, xx + 52, xx + 55);
  xx[52] = xx[11] * (xx[55] - xx[39] * xx[24]);
  xx[53] = (xx[56] - xx[39] * xx[25]) * xx[11] - xx[6];
  xx[54] = xx[11] * (xx[57] + xx[6] * xx[39] * xx[40]) - xx[17];
  pm_math_Quaternion_inverseXform_ra(xx + 1, xx + 52, xx + 55);
  pm_math_Vector3_cross_ra(xx + 55, xx + 46, xx + 58);
  pm_math_Quaternion_xform_ra(xx + 35, xx + 58, xx + 55);
  pm_math_Vector3_cross_ra(xx + 52, xx + 49, xx + 58);
  pm_math_Quaternion_xform_ra(xx + 39, xx + 58, xx + 52);
  xx[58] = - 0.02547118541792448;
  xx[59] = 3.457425605560157e-4;
  xx[60] = - 0.9996754969366247;
  pm_math_Quaternion_inverseXform_ra(xx + 39, xx + 58, xx + 61);
  pm_math_Quaternion_inverseXform_ra(xx + 1, xx + 61, xx + 58);
  pm_math_Vector3_cross_ra(xx + 58, xx + 46, xx + 1);
  pm_math_Quaternion_xform_ra(xx + 35, xx + 1, xx + 58);
  pm_math_Vector3_cross_ra(xx + 61, xx + 49, xx + 1);
  pm_math_Quaternion_xform_ra(xx + 39, xx + 1, xx + 35);
  xx[1] = motionData[14];
  xx[2] = motionData[15];
  xx[3] = motionData[16];
  xx[4] = motionData[17];
  xx[6] = 0.9998343934839863;
  xx[17] = 0.0181985056119827;
  xx[24] = xx[6] * state[27] + xx[17] * state[29];
  xx[25] = xx[6] * state[28] - xx[17] * state[30];
  xx[38] = xx[6] * state[29] - xx[17] * state[27];
  xx[39] = xx[17] * state[28] + xx[6] * state[30];
  xx[61] = - xx[24];
  xx[62] = xx[25];
  xx[63] = xx[38];
  xx[64] = xx[39];
  pm_math_Quaternion_compose_ra(xx + 1, xx + 61, xx + 65);
  xx[6] = xx[38] * xx[38];
  xx[17] = xx[39] * xx[39];
  xx[40] = 1.0;
  xx[41] = xx[24] * xx[39];
  xx[42] = xx[25] * xx[38];
  xx[49] = xx[38] * xx[24];
  xx[50] = xx[25] * xx[39];
  xx[69] = (xx[6] + xx[17]) * xx[11] - xx[40];
  xx[70] = - ((xx[41] + xx[42]) * xx[11]);
  xx[71] = xx[11] * (xx[49] - xx[50]);
  pm_math_Vector3_cross_ra(xx + 69, xx + 46, xx + 72);
  pm_math_Quaternion_xform_ra(xx + 65, xx + 72, xx + 75);
  xx[72] = - 0.8116895113357588;
  xx[73] = - 0.2717;
  xx[74] = - 0.03913410103116464;
  pm_math_Vector3_cross_ra(xx + 69, xx + 72, xx + 78);
  pm_math_Quaternion_xform_ra(xx + 61, xx + 78, xx + 69);
  pm_math_Quaternion_xform_ra(xx + 1, xx + 69, xx + 78);
  xx[51] = xx[25] * xx[25];
  xx[69] = xx[25] * xx[24];
  xx[24] = xx[38] * xx[39];
  xx[81] = xx[11] * (xx[41] - xx[42]);
  xx[82] = (xx[17] + xx[51]) * xx[11] - xx[40];
  xx[83] = - ((xx[69] + xx[24]) * xx[11]);
  pm_math_Vector3_cross_ra(xx + 81, xx + 46, xx + 84);
  pm_math_Quaternion_xform_ra(xx + 65, xx + 84, xx + 87);
  pm_math_Vector3_cross_ra(xx + 81, xx + 72, xx + 84);
  pm_math_Quaternion_xform_ra(xx + 61, xx + 84, xx + 81);
  pm_math_Quaternion_xform_ra(xx + 1, xx + 81, xx + 84);
  xx[81] = - ((xx[49] + xx[50]) * xx[11]);
  xx[82] = xx[11] * (xx[69] - xx[24]);
  xx[83] = (xx[51] + xx[6]) * xx[11] - xx[40];
  pm_math_Vector3_cross_ra(xx + 81, xx + 46, xx + 38);
  pm_math_Quaternion_xform_ra(xx + 65, xx + 38, xx + 46);
  pm_math_Vector3_cross_ra(xx + 81, xx + 72, xx + 38);
  pm_math_Quaternion_xform_ra(xx + 61, xx + 38, xx + 49);
  pm_math_Quaternion_xform_ra(xx + 1, xx + 49, xx + 38);
  J[6] = (xx[20] - xx[13] * xx[9]) * xx[11];
  J[7] = (xx[26] - xx[12] * xx[13] * xx[15]) * xx[11] - xx[7];
  J[8] = xx[5] + xx[11] * (xx[32] - xx[13] * xx[19]);
  J[9] = - (xx[29] + xx[43]);
  J[10] = - (xx[55] + xx[52]);
  J[11] = - (xx[58] + xx[35]);
  J[12] = - (xx[75] + xx[78]);
  J[13] = - (xx[87] + xx[84]);
  J[14] = - (xx[46] + xx[38]);
  J[64] = xx[7] + xx[11] * (xx[21] + xx[13] * xx[10]);
  J[65] = (xx[13] * xx[18] + xx[27]) * xx[11];
  J[66] = xx[12] + (xx[5] * xx[13] * xx[16] + xx[33]) * xx[11];
  J[67] = - (xx[30] + xx[44]);
  J[68] = - (xx[56] + xx[53]);
  J[69] = - (xx[59] + xx[36]);
  J[70] = - (xx[76] + xx[79]);
  J[71] = - (xx[88] + xx[85]);
  J[72] = - (xx[47] + xx[39]);
  J[122] = (xx[7] * xx[13] * xx[14] + xx[22]) * xx[11] - xx[5];
  J[123] = xx[11] * (xx[28] + xx[13] * xx[8]) - xx[12];
  J[124] = (xx[13] * xx[23] + xx[34]) * xx[11];
  J[125] = - (xx[31] + xx[45]);
  J[126] = - (xx[57] + xx[54]);
  J[127] = - (xx[60] + xx[37]);
  J[128] = - (xx[77] + xx[80]);
  J[129] = - (xx[89] + xx[86]);
  J[130] = - (xx[48] + xx[40]);
  return 3;
}

static size_t computeAssemblyJacobian_14(const RuntimeDerivedValuesBundle *rtdv,
  const double *state, const int *modeVector, const double *motionData, double
  *J)
{
  const double *rtdvd = rtdv->mDoubles.mValues;
  const int *rtdvi = rtdv->mInts.mValues;
  double xx[90];
  (void) rtdvd;
  (void) rtdvi;
  (void) modeVector;
  xx[0] = 0.0;
  xx[1] = 0.9999188709407687;
  xx[2] = 1.729133611399561e-4;
  xx[3] = 0.01273662563857442;
  xx[4] = 2.202511436425175e-6;
  xx[5] = - xx[1];
  xx[6] = - xx[2];
  xx[7] = - xx[3];
  xx[8] = - xx[4];
  xx[9] = - state[13];
  xx[10] = - state[14];
  xx[11] = - state[15];
  xx[12] = - state[16];
  pm_math_Quaternion_compose_ra(xx + 5, xx + 9, xx + 13);
  xx[5] = 0.29499;
  xx[6] = xx[5] * xx[15];
  xx[7] = 2.200000000000003e-3;
  xx[8] = xx[7] * xx[16];
  xx[9] = xx[6] + xx[8];
  xx[10] = xx[5] * xx[14];
  xx[17] = - xx[9];
  xx[18] = xx[10];
  xx[19] = xx[7] * xx[14];
  pm_math_Vector3_cross_ra(xx + 14, xx + 17, xx + 20);
  xx[11] = 2.0;
  xx[12] = 0.15472;
  xx[17] = xx[12] * xx[14];
  xx[18] = xx[8] + xx[17];
  xx[8] = xx[7] * xx[15];
  xx[23] = xx[12] * xx[15];
  xx[24] = - xx[18];
  xx[25] = xx[8];
  pm_math_Vector3_cross_ra(xx + 14, xx + 23, xx + 26);
  xx[19] = xx[12] * xx[16];
  xx[23] = xx[17] + xx[6];
  xx[29] = xx[19];
  xx[30] = xx[5] * xx[16];
  xx[31] = - xx[23];
  pm_math_Vector3_cross_ra(xx + 14, xx + 29, xx + 32);
  xx[35] = state[20];
  xx[36] = - state[21];
  xx[37] = - state[22];
  xx[38] = - state[23];
  pm_math_Quaternion_compose_ra(xx + 1, xx + 35, xx + 39);
  xx[1] = motionData[21];
  xx[2] = motionData[22];
  xx[3] = motionData[23];
  xx[4] = motionData[24];
  pm_math_Quaternion_compose_ra(xx + 39, xx + 1, xx + 35);
  xx[29] = - 0.9996755567249835;
  xx[30] = - 8.80933099498327e-6;
  xx[31] = 0.02547118389454986;
  pm_math_Quaternion_inverseXform_ra(xx + 39, xx + 29, xx + 43);
  pm_math_Quaternion_inverseXform_ra(xx + 1, xx + 43, xx + 29);
  xx[46] = 0.5299620178672857;
  xx[47] = 0.268;
  xx[48] = - 0.08339940224027326;
  pm_math_Vector3_cross_ra(xx + 29, xx + 46, xx + 49);
  pm_math_Quaternion_xform_ra(xx + 35, xx + 49, xx + 29);
  xx[49] = motionData[25];
  xx[50] = motionData[26];
  xx[51] = motionData[27];
  pm_math_Vector3_cross_ra(xx + 43, xx + 49, xx + 52);
  pm_math_Quaternion_xform_ra(xx + 39, xx + 52, xx + 43);
  xx[6] = 0.9999999401922369;
  xx[17] = 3.458547708105375e-4;
  xx[24] = xx[6] * xx[42] - xx[17] * xx[41];
  xx[25] = xx[17] * xx[40];
  xx[52] = xx[24];
  xx[53] = xx[25];
  xx[54] = - (xx[6] * xx[40]);
  pm_math_Vector3_cross_ra(xx + 40, xx + 52, xx + 55);
  xx[52] = xx[11] * (xx[55] - xx[39] * xx[24]);
  xx[53] = (xx[56] - xx[39] * xx[25]) * xx[11] - xx[6];
  xx[54] = xx[11] * (xx[57] + xx[6] * xx[39] * xx[40]) - xx[17];
  pm_math_Quaternion_inverseXform_ra(xx + 1, xx + 52, xx + 55);
  pm_math_Vector3_cross_ra(xx + 55, xx + 46, xx + 58);
  pm_math_Quaternion_xform_ra(xx + 35, xx + 58, xx + 55);
  pm_math_Vector3_cross_ra(xx + 52, xx + 49, xx + 58);
  pm_math_Quaternion_xform_ra(xx + 39, xx + 58, xx + 52);
  xx[58] = - 0.02547118541792448;
  xx[59] = 3.457425605560157e-4;
  xx[60] = - 0.9996754969366247;
  pm_math_Quaternion_inverseXform_ra(xx + 39, xx + 58, xx + 61);
  pm_math_Quaternion_inverseXform_ra(xx + 1, xx + 61, xx + 58);
  pm_math_Vector3_cross_ra(xx + 58, xx + 46, xx + 1);
  pm_math_Quaternion_xform_ra(xx + 35, xx + 1, xx + 58);
  pm_math_Vector3_cross_ra(xx + 61, xx + 49, xx + 1);
  pm_math_Quaternion_xform_ra(xx + 39, xx + 1, xx + 35);
  xx[1] = motionData[14];
  xx[2] = motionData[15];
  xx[3] = motionData[16];
  xx[4] = motionData[17];
  xx[6] = 0.9998343934839863;
  xx[17] = 0.0181985056119827;
  xx[24] = xx[6] * state[27] + xx[17] * state[29];
  xx[25] = xx[6] * state[28] - xx[17] * state[30];
  xx[38] = xx[6] * state[29] - xx[17] * state[27];
  xx[39] = xx[17] * state[28] + xx[6] * state[30];
  xx[61] = - xx[24];
  xx[62] = xx[25];
  xx[63] = xx[38];
  xx[64] = xx[39];
  pm_math_Quaternion_compose_ra(xx + 1, xx + 61, xx + 65);
  xx[6] = xx[38] * xx[38];
  xx[17] = xx[39] * xx[39];
  xx[40] = 1.0;
  xx[41] = xx[24] * xx[39];
  xx[42] = xx[25] * xx[38];
  xx[49] = xx[38] * xx[24];
  xx[50] = xx[25] * xx[39];
  xx[69] = (xx[6] + xx[17]) * xx[11] - xx[40];
  xx[70] = - ((xx[41] + xx[42]) * xx[11]);
  xx[71] = xx[11] * (xx[49] - xx[50]);
  pm_math_Vector3_cross_ra(xx + 69, xx + 46, xx + 72);
  pm_math_Quaternion_xform_ra(xx + 65, xx + 72, xx + 75);
  xx[72] = - 0.8116895113357588;
  xx[73] = - 0.2717;
  xx[74] = - 0.03913410103116464;
  pm_math_Vector3_cross_ra(xx + 69, xx + 72, xx + 78);
  pm_math_Quaternion_xform_ra(xx + 61, xx + 78, xx + 69);
  pm_math_Quaternion_xform_ra(xx + 1, xx + 69, xx + 78);
  xx[51] = xx[25] * xx[25];
  xx[69] = xx[25] * xx[24];
  xx[24] = xx[38] * xx[39];
  xx[81] = xx[11] * (xx[41] - xx[42]);
  xx[82] = (xx[17] + xx[51]) * xx[11] - xx[40];
  xx[83] = - ((xx[69] + xx[24]) * xx[11]);
  pm_math_Vector3_cross_ra(xx + 81, xx + 46, xx + 84);
  pm_math_Quaternion_xform_ra(xx + 65, xx + 84, xx + 87);
  pm_math_Vector3_cross_ra(xx + 81, xx + 72, xx + 84);
  pm_math_Quaternion_xform_ra(xx + 61, xx + 84, xx + 81);
  pm_math_Quaternion_xform_ra(xx + 1, xx + 81, xx + 84);
  xx[81] = - ((xx[49] + xx[50]) * xx[11]);
  xx[82] = xx[11] * (xx[69] - xx[24]);
  xx[83] = (xx[51] + xx[6]) * xx[11] - xx[40];
  pm_math_Vector3_cross_ra(xx + 81, xx + 46, xx + 38);
  pm_math_Quaternion_xform_ra(xx + 65, xx + 38, xx + 46);
  pm_math_Vector3_cross_ra(xx + 81, xx + 72, xx + 38);
  pm_math_Quaternion_xform_ra(xx + 61, xx + 38, xx + 49);
  pm_math_Quaternion_xform_ra(xx + 1, xx + 49, xx + 38);
  J[6] = (xx[20] - xx[13] * xx[9]) * xx[11];
  J[7] = (xx[12] * xx[13] * xx[15] + xx[26]) * xx[11] - xx[7];
  J[8] = xx[5] + xx[11] * (xx[32] + xx[13] * xx[19]);
  J[9] = - (xx[29] + xx[43]);
  J[10] = - (xx[55] + xx[52]);
  J[11] = - (xx[58] + xx[35]);
  J[12] = - (xx[75] + xx[78]);
  J[13] = - (xx[87] + xx[84]);
  J[14] = - (xx[46] + xx[38]);
  J[64] = xx[7] + xx[11] * (xx[21] + xx[13] * xx[10]);
  J[65] = (xx[27] - xx[13] * xx[18]) * xx[11];
  J[66] = (xx[5] * xx[13] * xx[16] + xx[33]) * xx[11] - xx[12];
  J[67] = - (xx[30] + xx[44]);
  J[68] = - (xx[56] + xx[53]);
  J[69] = - (xx[59] + xx[36]);
  J[70] = - (xx[76] + xx[79]);
  J[71] = - (xx[88] + xx[85]);
  J[72] = - (xx[47] + xx[39]);
  J[122] = (xx[7] * xx[13] * xx[14] + xx[22]) * xx[11] - xx[5];
  J[123] = xx[12] + xx[11] * (xx[28] + xx[13] * xx[8]);
  J[124] = (xx[34] - xx[13] * xx[23]) * xx[11];
  J[125] = - (xx[31] + xx[45]);
  J[126] = - (xx[57] + xx[54]);
  J[127] = - (xx[60] + xx[37]);
  J[128] = - (xx[77] + xx[80]);
  J[129] = - (xx[89] + xx[86]);
  J[130] = - (xx[48] + xx[40]);
  return 3;
}

static size_t computeAssemblyJacobian_15(const RuntimeDerivedValuesBundle *rtdv,
  const double *state, const int *modeVector, const double *motionData, double
  *J)
{
  const double *rtdvd = rtdv->mDoubles.mValues;
  const int *rtdvi = rtdv->mInts.mValues;
  double xx[48];
  (void) rtdvd;
  (void) rtdvi;
  (void) modeVector;
  (void) motionData;
  xx[0] = 0.0;
  xx[1] = 0.9998343934839863;
  xx[2] = 0.0181985056119827;
  xx[3] = xx[1] * state[100] - xx[2] * state[98];
  xx[4] = 0.26683;
  xx[5] = xx[3] * xx[4];
  xx[6] = 0.04444999999999999;
  xx[7] = xx[1] * state[101] + xx[2] * state[99];
  xx[8] = xx[6] * xx[7];
  xx[9] = xx[5] - xx[8];
  xx[10] = xx[1] * state[98] + xx[2] * state[100];
  xx[11] = xx[1] * state[99] - xx[2] * state[101];
  xx[12] = xx[11];
  xx[13] = xx[3];
  xx[14] = xx[7];
  xx[1] = xx[11] * xx[4];
  xx[2] = xx[11] * xx[6];
  xx[15] = xx[9];
  xx[16] = - xx[1];
  xx[17] = xx[2];
  pm_math_Vector3_cross_ra(xx + 12, xx + 15, xx + 18);
  xx[15] = 2.0;
  xx[16] = 0.05344999999999999;
  xx[17] = xx[16] * xx[7];
  xx[21] = xx[17] - xx[5];
  xx[22] = xx[11] * xx[16];
  xx[23] = xx[21];
  xx[24] = xx[1];
  xx[25] = - xx[22];
  pm_math_Vector3_cross_ra(xx + 12, xx + 23, xx + 26);
  xx[23] = 0.14521;
  xx[24] = xx[3] * xx[23];
  xx[25] = xx[11] * xx[23];
  xx[29] = xx[8] + xx[25];
  xx[8] = xx[3] * xx[6];
  xx[30] = xx[24];
  xx[31] = - xx[29];
  xx[32] = xx[8];
  pm_math_Vector3_cross_ra(xx + 12, xx + 30, xx + 33);
  xx[6] = 0.14279;
  xx[30] = xx[3] * xx[6];
  xx[31] = xx[11] * xx[6];
  xx[11] = xx[17] - xx[31];
  xx[17] = xx[3] * xx[16];
  xx[36] = xx[30];
  xx[37] = xx[11];
  xx[38] = - xx[17];
  pm_math_Vector3_cross_ra(xx + 12, xx + 36, xx + 39);
  xx[3] = 9.000000000000001e-3;
  xx[16] = xx[23] * xx[7];
  xx[23] = xx[4] * xx[7];
  xx[4] = xx[5] - xx[25];
  xx[36] = xx[16];
  xx[37] = - xx[23];
  xx[38] = xx[4];
  pm_math_Vector3_cross_ra(xx + 12, xx + 36, xx + 42);
  xx[25] = xx[6] * xx[7];
  xx[6] = xx[31] + xx[5];
  xx[36] = xx[25];
  xx[37] = xx[23];
  xx[38] = - xx[6];
  pm_math_Vector3_cross_ra(xx + 12, xx + 36, xx + 45);
  xx[5] = xx[1] * xx[10];
  xx[1] = xx[23] * xx[10];
  xx[7] = 0.288;
  J[43] = (xx[9] * xx[10] + xx[18]) * xx[15] + (xx[10] * xx[21] + xx[26]) * xx
    [15];
  J[44] = (xx[24] * xx[10] + xx[33]) * xx[15] + (xx[30] * xx[10] + xx[39]) * xx
    [15] + xx[3];
  J[45] = xx[15] * (xx[42] + xx[16] * xx[10]) + xx[15] * (xx[45] + xx[25] * xx
    [10]);
  J[101] = xx[15] * (xx[19] - xx[5]) + xx[15] * (xx[27] + xx[5]) - xx[3];
  J[102] = (xx[34] - xx[10] * xx[29]) * xx[15] + (xx[11] * xx[10] + xx[40]) *
    xx[15];
  J[103] = (xx[43] - xx[1]) * xx[15] + (xx[1] + xx[46]) * xx[15] - xx[7];
  J[159] = (xx[2] * xx[10] + xx[20]) * xx[15] + (xx[28] - xx[22] * xx[10]) * xx
    [15];
  J[160] = xx[15] * (xx[35] + xx[8] * xx[10]) + xx[15] * (xx[41] - xx[17] * xx
    [10]) + xx[7];
  J[161] = (xx[10] * xx[4] + xx[44]) * xx[15] + (xx[47] - xx[10] * xx[6]) * xx
    [15];
  return 3;
}

static size_t computeAssemblyJacobian_16(const RuntimeDerivedValuesBundle *rtdv,
  const double *state, const int *modeVector, const double *motionData, double
  *J)
{
  const double *rtdvd = rtdv->mDoubles.mValues;
  const int *rtdvi = rtdv->mInts.mValues;
  double xx[37];
  (void) rtdvd;
  (void) rtdvi;
  (void) modeVector;
  (void) motionData;
  xx[0] = 0.0;
  xx[1] = 0.9998343934839863;
  xx[2] = 0.0181985056119827;
  xx[3] = xx[1] * state[27] + xx[2] * state[29];
  xx[4] = xx[1] * state[28] - xx[2] * state[30];
  xx[5] = xx[1] * state[29] - xx[2] * state[27];
  xx[6] = xx[2] * state[28] + xx[1] * state[30];
  xx[7] = - xx[3];
  xx[8] = xx[4];
  xx[9] = xx[5];
  xx[10] = xx[6];
  xx[1] = xx[5] * xx[5];
  xx[2] = xx[6] * xx[6];
  xx[11] = 2.0;
  xx[12] = 1.0;
  xx[13] = xx[3] * xx[6];
  xx[14] = xx[4] * xx[5];
  xx[15] = xx[5] * xx[3];
  xx[16] = xx[4] * xx[6];
  xx[17] = (xx[1] + xx[2]) * xx[11] - xx[12];
  xx[18] = - ((xx[13] + xx[14]) * xx[11]);
  xx[19] = xx[11] * (xx[15] - xx[16]);
  xx[20] = 0.2717;
  xx[21] = 0.5235527553923325;
  xx[22] = xx[20];
  xx[23] = 0.03764753640142538;
  pm_math_Vector3_cross_ra(xx + 17, xx + 21, xx + 24);
  pm_math_Quaternion_xform_ra(xx + 7, xx + 24, xx + 27);
  xx[24] = - 0.8116895113357588;
  xx[25] = - xx[20];
  xx[26] = - 0.03913410103116464;
  pm_math_Vector3_cross_ra(xx + 17, xx + 24, xx + 30);
  pm_math_Quaternion_xform_ra(xx + 7, xx + 30, xx + 17);
  xx[20] = xx[4] * xx[4];
  xx[30] = xx[4] * xx[3];
  xx[3] = xx[5] * xx[6];
  xx[4] = xx[11] * (xx[13] - xx[14]);
  xx[5] = (xx[2] + xx[20]) * xx[11] - xx[12];
  xx[6] = - ((xx[30] + xx[3]) * xx[11]);
  pm_math_Vector3_cross_ra(xx + 4, xx + 21, xx + 31);
  pm_math_Quaternion_xform_ra(xx + 7, xx + 31, xx + 34);
  pm_math_Vector3_cross_ra(xx + 4, xx + 24, xx + 31);
  pm_math_Quaternion_xform_ra(xx + 7, xx + 31, xx + 4);
  xx[31] = - ((xx[15] + xx[16]) * xx[11]);
  xx[32] = xx[11] * (xx[30] - xx[3]);
  xx[33] = (xx[20] + xx[1]) * xx[11] - xx[12];
  pm_math_Vector3_cross_ra(xx + 31, xx + 21, xx + 1);
  pm_math_Quaternion_xform_ra(xx + 7, xx + 1, xx + 11);
  pm_math_Vector3_cross_ra(xx + 31, xx + 24, xx + 1);
  pm_math_Quaternion_xform_ra(xx + 7, xx + 1, xx + 14);
  J[12] = - (xx[27] + xx[17]);
  J[13] = - (xx[34] + xx[4]);
  J[14] = - (xx[11] + xx[14]);
  J[70] = - (xx[28] + xx[18]);
  J[71] = - (xx[35] + xx[5]);
  J[72] = - (xx[12] + xx[15]);
  J[128] = - (xx[29] + xx[19]);
  J[129] = - (xx[36] + xx[6]);
  J[130] = - (xx[13] + xx[16]);
  return 3;
}

static size_t computeAssemblyJacobian_17(const RuntimeDerivedValuesBundle *rtdv,
  const double *state, const int *modeVector, const double *motionData, double
  *J)
{
  const double *rtdvd = rtdv->mDoubles.mValues;
  const int *rtdvi = rtdv->mInts.mValues;
  double xx[67];
  (void) rtdvd;
  (void) rtdvi;
  (void) modeVector;
  xx[0] = 0.0;
  xx[1] = 0.9998343934839863;
  xx[2] = 0.0181985056119827;
  xx[3] = xx[1] * state[27] + xx[2] * state[29];
  xx[4] = xx[1] * state[28] - xx[2] * state[30];
  xx[5] = xx[1] * state[29] - xx[2] * state[27];
  xx[6] = xx[2] * state[28] + xx[1] * state[30];
  xx[7] = - xx[3];
  xx[8] = xx[4];
  xx[9] = xx[5];
  xx[10] = xx[6];
  xx[11] = motionData[98];
  xx[12] = motionData[99];
  xx[13] = motionData[100];
  xx[14] = motionData[101];
  pm_math_Quaternion_compose_ra(xx + 7, xx + 11, xx + 15);
  xx[1] = 3.749811151839773e-3;
  xx[2] = xx[5] * xx[5];
  xx[19] = xx[6] * xx[6];
  xx[20] = 2.0;
  xx[21] = 1.0;
  xx[22] = xx[3] * xx[6];
  xx[23] = xx[4] * xx[5];
  xx[24] = xx[5] * xx[3];
  xx[25] = xx[4] * xx[6];
  xx[26] = (xx[2] + xx[19]) * xx[20] - xx[21];
  xx[27] = - ((xx[22] + xx[23]) * xx[20]);
  xx[28] = xx[20] * (xx[24] - xx[25]);
  pm_math_Quaternion_inverseXform_ra(xx + 11, xx + 26, xx + 29);
  xx[32] = 0.05742288587815435;
  xx[33] = xx[1] * xx[30] - xx[32] * xx[31];
  xx[34] = - (xx[1] * xx[29]);
  xx[35] = xx[32] * xx[29];
  pm_math_Quaternion_xform_ra(xx + 15, xx + 33, xx + 29);
  xx[33] = motionData[102];
  xx[34] = motionData[103];
  xx[35] = motionData[104];
  pm_math_Vector3_cross_ra(xx + 26, xx + 33, xx + 36);
  pm_math_Quaternion_xform_ra(xx + 7, xx + 36, xx + 39);
  xx[36] = - 0.8116895113357588;
  xx[37] = - 0.2717;
  xx[38] = - 0.03913410103116464;
  pm_math_Vector3_cross_ra(xx + 26, xx + 36, xx + 42);
  pm_math_Quaternion_xform_ra(xx + 7, xx + 42, xx + 26);
  xx[42] = xx[4] * xx[4];
  xx[43] = xx[4] * xx[3];
  xx[3] = xx[5] * xx[6];
  xx[4] = xx[20] * (xx[22] - xx[23]);
  xx[5] = (xx[19] + xx[42]) * xx[20] - xx[21];
  xx[6] = - ((xx[43] + xx[3]) * xx[20]);
  pm_math_Quaternion_inverseXform_ra(xx + 11, xx + 4, xx + 44);
  xx[47] = xx[1] * xx[45] - xx[32] * xx[46];
  xx[48] = - (xx[1] * xx[44]);
  xx[49] = xx[32] * xx[44];
  pm_math_Quaternion_xform_ra(xx + 15, xx + 47, xx + 44);
  pm_math_Vector3_cross_ra(xx + 4, xx + 33, xx + 47);
  pm_math_Quaternion_xform_ra(xx + 7, xx + 47, xx + 50);
  pm_math_Vector3_cross_ra(xx + 4, xx + 36, xx + 47);
  pm_math_Quaternion_xform_ra(xx + 7, xx + 47, xx + 4);
  xx[47] = - ((xx[24] + xx[25]) * xx[20]);
  xx[48] = xx[20] * (xx[43] - xx[3]);
  xx[49] = (xx[42] + xx[2]) * xx[20] - xx[21];
  pm_math_Quaternion_inverseXform_ra(xx + 11, xx + 47, xx + 21);
  xx[11] = xx[1] * xx[22] - xx[32] * xx[23];
  xx[12] = - (xx[1] * xx[21]);
  xx[13] = xx[32] * xx[21];
  pm_math_Quaternion_xform_ra(xx + 15, xx + 11, xx + 21);
  pm_math_Vector3_cross_ra(xx + 47, xx + 33, xx + 11);
  pm_math_Quaternion_xform_ra(xx + 7, xx + 11, xx + 14);
  pm_math_Vector3_cross_ra(xx + 47, xx + 36, xx + 11);
  pm_math_Quaternion_xform_ra(xx + 7, xx + 11, xx + 17);
  xx[7] = motionData[21];
  xx[8] = motionData[22];
  xx[9] = motionData[23];
  xx[10] = motionData[24];
  xx[2] = 0.9996758317937826;
  xx[3] = 0.5 * state[94];
  xx[11] = cos(xx[3]);
  xx[12] = 0.01780577773926495;
  xx[13] = sin(xx[3]);
  xx[3] = xx[2] * xx[11] + xx[12] * xx[13];
  xx[24] = xx[2] * xx[13] - xx[12] * xx[11];
  xx[2] = 0.01819561954822274;
  xx[12] = 3.240922178968028e-4;
  xx[25] = xx[2] * xx[11] + xx[12] * xx[13];
  xx[33] = - xx[25];
  xx[34] = xx[2] * xx[13] - xx[12] * xx[11];
  xx[35] = xx[3];
  xx[36] = xx[24];
  xx[37] = xx[33];
  xx[38] = xx[34];
  pm_math_Quaternion_compose_ra(xx + 7, xx + 35, xx + 53);
  xx[2] = xx[32] * xx[55] + xx[1] * xx[56];
  xx[11] = xx[32] * xx[54];
  xx[35] = xx[2];
  xx[36] = - xx[11];
  xx[37] = - (xx[1] * xx[54]);
  pm_math_Vector3_cross_ra(xx + 54, xx + 35, xx + 47);
  xx[12] = 5.002590631222067e-3;
  xx[13] = 0.05733821843436025;
  xx[35] = xx[12] * xx[34] - xx[25] * xx[13];
  xx[36] = xx[24];
  xx[37] = xx[33];
  xx[38] = xx[34];
  xx[25] = xx[24] * xx[13];
  xx[33] = xx[24] * xx[12];
  xx[55] = xx[35];
  xx[56] = - xx[25];
  xx[57] = - xx[33];
  pm_math_Vector3_cross_ra(xx + 36, xx + 55, xx + 58);
  xx[36] = (xx[35] * xx[3] + xx[58]) * xx[20];
  xx[37] = xx[20] * (xx[59] - xx[25] * xx[3]) - xx[12];
  xx[38] = xx[13] + (xx[60] - xx[33] * xx[3]) * xx[20];
  pm_math_Quaternion_xform_ra(xx + 7, xx + 36, xx + 33);
  xx[7] = state[127];
  xx[8] = state[128];
  xx[9] = state[129];
  xx[3] = 0.17753;
  xx[10] = xx[3] * state[128];
  xx[12] = 0.15579;
  xx[13] = xx[12] * state[129];
  xx[24] = xx[10] + xx[13];
  xx[25] = xx[3] * state[127];
  xx[36] = - xx[24];
  xx[37] = xx[25];
  xx[38] = xx[12] * state[127];
  pm_math_Vector3_cross_ra(xx + 7, xx + 36, xx + 55);
  xx[36] = 5.789999999999964e-3;
  xx[37] = xx[36] * state[127];
  xx[38] = xx[37] - xx[13];
  xx[13] = xx[12] * state[128];
  xx[58] = - (xx[36] * state[128]);
  xx[59] = xx[38];
  xx[60] = xx[13];
  pm_math_Vector3_cross_ra(xx + 7, xx + 58, xx + 61);
  xx[42] = xx[36] * state[129];
  xx[43] = xx[37] - xx[10];
  xx[58] = - xx[42];
  xx[59] = xx[3] * state[129];
  xx[60] = xx[43];
  pm_math_Vector3_cross_ra(xx + 7, xx + 58, xx + 64);
  J[12] = - (xx[29] + xx[39] + xx[26]);
  J[13] = - (xx[44] + xx[50] + xx[4]);
  J[14] = - (xx[21] + xx[14] + xx[17]);
  J[41] = - ((xx[53] * xx[2] + xx[47]) * xx[20] + xx[33]);
  J[55] = (xx[55] - xx[24] * state[126]) * xx[20];
  J[56] = (xx[61] - xx[36] * state[126] * state[128]) * xx[20] - xx[12];
  J[57] = xx[3] + xx[20] * (xx[64] - xx[42] * state[126]);
  J[70] = - (xx[30] + xx[40] + xx[27]);
  J[71] = - (xx[45] + xx[51] + xx[5]);
  J[72] = - (xx[22] + xx[15] + xx[18]);
  J[99] = - (xx[20] * (xx[48] - xx[53] * xx[11]) + xx[34] - xx[1]);
  J[113] = xx[12] + xx[20] * (xx[56] + xx[25] * state[126]);
  J[114] = (xx[38] * state[126] + xx[62]) * xx[20];
  J[115] = xx[36] + (xx[3] * state[126] * state[129] + xx[65]) * xx[20];
  J[128] = - (xx[31] + xx[41] + xx[28]);
  J[129] = - (xx[46] + xx[52] + xx[6]);
  J[130] = - (xx[23] + xx[16] + xx[19]);
  J[157] = - ((xx[49] - xx[1] * xx[53] * xx[54]) * xx[20] + xx[35] + xx[32]);
  J[171] = (xx[12] * state[126] * state[127] + xx[57]) * xx[20] - xx[3];
  J[172] = xx[20] * (xx[63] + xx[13] * state[126]) - xx[36];
  J[173] = (xx[43] * state[126] + xx[66]) * xx[20];
  return 3;
}

static size_t computeAssemblyJacobian_18(const RuntimeDerivedValuesBundle *rtdv,
  const double *state, const int *modeVector, const double *motionData, double
  *J)
{
  const double *rtdvd = rtdv->mDoubles.mValues;
  const int *rtdvi = rtdv->mInts.mValues;
  double xx[80];
  (void) rtdvd;
  (void) rtdvi;
  (void) modeVector;
  xx[0] = 0.0;
  xx[1] = 0.9996758317937826;
  xx[2] = 0.5 * state[96];
  xx[3] = cos(xx[2]);
  xx[4] = 0.01780577773926495;
  xx[5] = sin(xx[2]);
  xx[2] = xx[1] * xx[3] - xx[4] * xx[5];
  xx[6] = 3.749811151839773e-3;
  xx[7] = 3.240922178968028e-4;
  xx[8] = 0.01819561954822274;
  xx[9] = xx[7] * xx[3] + xx[8] * xx[5];
  xx[10] = xx[7] * xx[5] - xx[8] * xx[3];
  xx[7] = 0.05742288587815435;
  xx[8] = xx[6] * xx[9] - xx[10] * xx[7];
  xx[11] = xx[1] * xx[5] + xx[4] * xx[3];
  xx[3] = xx[11];
  xx[4] = xx[10];
  xx[5] = xx[9];
  xx[1] = xx[11] * xx[7];
  xx[7] = xx[11] * xx[6];
  xx[12] = xx[8];
  xx[13] = xx[1];
  xx[14] = - xx[7];
  pm_math_Vector3_cross_ra(xx + 3, xx + 12, xx + 15);
  xx[6] = 2.0;
  xx[12] = 5.002590631222067e-3;
  xx[13] = 0.05733821843436025;
  xx[14] = xx[12] * xx[9] - xx[10] * xx[13];
  xx[9] = xx[11] * xx[13];
  xx[10] = xx[11] * xx[12];
  xx[11] = xx[14];
  xx[12] = xx[9];
  xx[13] = - xx[10];
  pm_math_Vector3_cross_ra(xx + 3, xx + 11, xx + 18);
  xx[3] = 0.9998343934839863;
  xx[4] = 0.0181985056119827;
  xx[5] = xx[3] * state[100] - xx[4] * state[98];
  xx[11] = xx[5] * motionData[124];
  xx[12] = xx[3] * state[101] + xx[4] * state[99];
  xx[13] = xx[12] * motionData[125];
  xx[21] = xx[11] + xx[13];
  xx[22] = xx[3] * state[98] + xx[4] * state[100];
  xx[23] = xx[3] * state[99] - xx[4] * state[101];
  xx[24] = xx[23];
  xx[25] = xx[5];
  xx[26] = xx[12];
  xx[3] = xx[23] * motionData[124];
  xx[4] = xx[23] * motionData[125];
  xx[27] = xx[21];
  xx[28] = - xx[3];
  xx[29] = - xx[4];
  pm_math_Vector3_cross_ra(xx + 24, xx + 27, xx + 30);
  xx[27] = 0.05344999999999999;
  xx[28] = xx[27] * xx[12];
  xx[29] = 0.26683;
  xx[33] = xx[5] * xx[29];
  xx[34] = xx[28] - xx[33];
  xx[35] = xx[23] * xx[29];
  xx[36] = xx[23] * xx[27];
  xx[37] = xx[34];
  xx[38] = xx[35];
  xx[39] = - xx[36];
  pm_math_Vector3_cross_ra(xx + 24, xx + 37, xx + 40);
  xx[37] = xx[5] * motionData[123];
  xx[38] = xx[23] * motionData[123];
  xx[39] = xx[13] + xx[38];
  xx[13] = xx[5] * motionData[125];
  xx[43] = - xx[37];
  xx[44] = xx[39];
  xx[45] = - xx[13];
  pm_math_Vector3_cross_ra(xx + 24, xx + 43, xx + 46);
  xx[43] = 0.14279;
  xx[44] = xx[5] * xx[43];
  xx[45] = xx[23] * xx[43];
  xx[23] = xx[28] - xx[45];
  xx[28] = xx[5] * xx[27];
  xx[49] = xx[44];
  xx[50] = xx[23];
  xx[51] = - xx[28];
  pm_math_Vector3_cross_ra(xx + 24, xx + 49, xx + 52);
  xx[5] = xx[12] * motionData[123];
  xx[49] = xx[12] * motionData[124];
  xx[50] = xx[38] + xx[11];
  xx[55] = - xx[5];
  xx[56] = - xx[49];
  xx[57] = xx[50];
  pm_math_Vector3_cross_ra(xx + 24, xx + 55, xx + 58);
  xx[11] = xx[43] * xx[12];
  xx[38] = xx[29] * xx[12];
  xx[12] = xx[45] + xx[33];
  xx[55] = xx[11];
  xx[56] = xx[38];
  xx[57] = - xx[12];
  pm_math_Vector3_cross_ra(xx + 24, xx + 55, xx + 61);
  xx[64] = motionData[112];
  xx[65] = motionData[113];
  xx[66] = motionData[114];
  xx[67] = motionData[115];
  xx[24] = 0.17753;
  xx[25] = xx[24] * state[107];
  xx[26] = 0.15579;
  xx[33] = xx[26] * state[108];
  xx[45] = xx[25] - xx[33];
  xx[55] = state[106];
  xx[56] = state[107];
  xx[57] = state[108];
  xx[51] = xx[24] * state[106];
  xx[68] = xx[45];
  xx[69] = - xx[51];
  xx[70] = xx[26] * state[106];
  pm_math_Vector3_cross_ra(xx + 55, xx + 68, xx + 71);
  xx[68] = (xx[45] * state[105] + xx[71]) * xx[6];
  xx[69] = xx[26] + xx[6] * (xx[72] - xx[51] * state[105]);
  xx[70] = xx[24] + (xx[26] * state[105] * state[106] + xx[73]) * xx[6];
  pm_math_Quaternion_xform_ra(xx + 64, xx + 68, xx + 71);
  xx[45] = 5.789999999999964e-3;
  xx[51] = xx[45] * state[106];
  xx[68] = xx[51] - xx[33];
  xx[33] = xx[26] * state[107];
  xx[74] = - (xx[45] * state[107]);
  xx[75] = xx[68];
  xx[76] = xx[33];
  pm_math_Vector3_cross_ra(xx + 55, xx + 74, xx + 77);
  xx[74] = (xx[77] - xx[45] * state[105] * state[107]) * xx[6] - xx[26];
  xx[75] = (xx[68] * state[105] + xx[78]) * xx[6];
  xx[76] = xx[6] * (xx[79] + xx[33] * state[105]) - xx[45];
  pm_math_Quaternion_xform_ra(xx + 64, xx + 74, xx + 68);
  xx[26] = xx[45] * state[108];
  xx[33] = xx[51] + xx[25];
  xx[74] = - xx[26];
  xx[75] = - (xx[24] * state[108]);
  xx[76] = xx[33];
  pm_math_Vector3_cross_ra(xx + 55, xx + 74, xx + 77);
  xx[55] = xx[6] * (xx[77] - xx[26] * state[105]) - xx[24];
  xx[56] = xx[45] + (xx[78] - xx[24] * state[105] * state[108]) * xx[6];
  xx[57] = (xx[33] * state[105] + xx[79]) * xx[6];
  pm_math_Quaternion_xform_ra(xx + 64, xx + 55, xx + 24);
  J[42] = - ((xx[2] * xx[8] + xx[15]) * xx[6] + (xx[2] * xx[14] + xx[18]) * xx[6]);
  J[43] = (xx[21] * xx[22] + xx[30]) * xx[6] + (xx[22] * xx[34] + xx[40]) * xx[6];
  J[44] = motionData[125] + xx[6] * (xx[46] - xx[37] * xx[22]) + (xx[44] * xx[22]
    + xx[52]) * xx[6] + xx[27];
  J[45] = xx[6] * (xx[58] - xx[5] * xx[22]) + xx[6] * (xx[61] + xx[11] * xx[22])
    - motionData[124] + xx[29];
  J[46] = xx[71];
  J[47] = xx[68];
  J[48] = xx[24];
  J[100] = - (xx[6] * (xx[16] + xx[1] * xx[2]) + xx[6] * (xx[19] + xx[9] * xx[2])
              - 8.752401783061839e-3);
  J[101] = xx[6] * (xx[31] - xx[3] * xx[22]) + xx[6] * (xx[41] + xx[35] * xx[22])
    - motionData[125] - xx[27];
  J[102] = (xx[39] * xx[22] + xx[47]) * xx[6] + (xx[23] * xx[22] + xx[53]) * xx
    [6];
  J[103] = motionData[123] + xx[6] * (xx[59] - xx[49] * xx[22]) + (xx[38] * xx
    [22] + xx[62]) * xx[6] - xx[43];
  J[104] = xx[72];
  J[105] = xx[69];
  J[106] = xx[25];
  J[158] = - ((xx[17] - xx[7] * xx[2]) * xx[6] + (xx[20] - xx[10] * xx[2]) * xx
              [6] - 0.1147611043125146);
  J[159] = motionData[124] + xx[6] * (xx[32] - xx[4] * xx[22]) + (xx[42] - xx[36]
    * xx[22]) * xx[6] - xx[29];
  J[160] = xx[6] * (xx[48] - xx[13] * xx[22]) + xx[6] * (xx[54] - xx[28] * xx[22])
    - motionData[123] + xx[43];
  J[161] = (xx[50] * xx[22] + xx[60]) * xx[6] + (xx[63] - xx[22] * xx[12]) * xx
    [6];
  J[162] = xx[73];
  J[163] = xx[70];
  J[164] = xx[26];
  return 3;
}

size_t dempsystest_bjorn_59d2bdba_5_computeAssemblyJacobian(const void *mech,
  const RuntimeDerivedValuesBundle *rtdv, size_t constraintIdx, boolean_T
  forVelocitySatisfaction, const double *state, const int *modeVector, const
  double *motionData, double *J)
{
  (void) mech;
  (void) rtdv;
  (void) state;
  (void) modeVector;
  (void) forVelocitySatisfaction;
  (void) motionData;
  (void) J;
  switch (constraintIdx)
  {
   case 0:
    return computeAssemblyJacobian_0(rtdv, state, modeVector, motionData, J);

   case 1:
    return computeAssemblyJacobian_1(rtdv, state, modeVector, motionData, J);

   case 2:
    return computeAssemblyJacobian_2(rtdv, state, modeVector, motionData, J);

   case 3:
    return computeAssemblyJacobian_3(rtdv, state, modeVector, motionData, J);

   case 4:
    return computeAssemblyJacobian_4(rtdv, state, modeVector, motionData, J);

   case 5:
    return computeAssemblyJacobian_5(rtdv, state, modeVector, motionData, J);

   case 6:
    return computeAssemblyJacobian_6(rtdv, state, modeVector, motionData, J);

   case 7:
    return computeAssemblyJacobian_7(rtdv, state, modeVector, motionData, J);

   case 8:
    return computeAssemblyJacobian_8(rtdv, state, modeVector, motionData, J);

   case 9:
    return computeAssemblyJacobian_9(rtdv, state, modeVector, motionData, J);

   case 10:
    return computeAssemblyJacobian_10(rtdv, state, modeVector, motionData, J);

   case 11:
    return computeAssemblyJacobian_11(rtdv, state, modeVector, motionData, J);

   case 12:
    return computeAssemblyJacobian_12(rtdv, state, modeVector, motionData, J);

   case 13:
    return computeAssemblyJacobian_13(rtdv, state, modeVector, motionData, J);

   case 14:
    return computeAssemblyJacobian_14(rtdv, state, modeVector, motionData, J);

   case 15:
    return computeAssemblyJacobian_15(rtdv, state, modeVector, motionData, J);

   case 16:
    return computeAssemblyJacobian_16(rtdv, state, modeVector, motionData, J);

   case 17:
    return computeAssemblyJacobian_17(rtdv, state, modeVector, motionData, J);

   case 18:
    return computeAssemblyJacobian_18(rtdv, state, modeVector, motionData, J);
  }

  return 0;
}

size_t dempsystest_bjorn_59d2bdba_5_computeFullAssemblyJacobian(const void *mech,
  const RuntimeDerivedValuesBundle *rtdv, const double *state, const int
  *modeVector, const double *motionData, double *J)
{
  const double *rtdvd = rtdv->mDoubles.mValues;
  const int *rtdvi = rtdv->mInts.mValues;
  double xx[668];
  (void) mech;
  (void) rtdvd;
  (void) rtdvi;
  (void) modeVector;
  xx[0] = 0.0;
  xx[1] = 0.9691346498545245;
  xx[2] = 0.5;
  xx[3] = xx[2] * state[34];
  xx[4] = cos(xx[3]);
  xx[5] = 0.2458594005622715;
  xx[6] = sin(xx[3]);
  xx[3] = xx[1] * xx[4] - xx[5] * xx[6];
  xx[7] = 0.06977021494021335;
  xx[8] = 4.475014772496788e-3;
  xx[9] = 0.01763972361731614;
  xx[10] = xx[8] * xx[4] + xx[9] * xx[6];
  xx[11] = xx[8] * xx[6] - xx[9] * xx[4];
  xx[12] = 3.56124026098071e-3;
  xx[13] = xx[7] * xx[10] - xx[11] * xx[12];
  xx[14] = xx[1] * xx[6] + xx[5] * xx[4];
  xx[15] = xx[14];
  xx[16] = xx[11];
  xx[17] = xx[10];
  xx[4] = xx[14] * xx[12];
  xx[6] = xx[14] * xx[7];
  xx[18] = xx[13];
  xx[19] = xx[4];
  xx[20] = - xx[6];
  pm_math_Vector3_cross_ra(xx + 15, xx + 18, xx + 21);
  xx[18] = 2.0;
  xx[19] = 0.02673029948304497;
  xx[20] = 0.03945399617968662;
  xx[24] = xx[11] * xx[19] + xx[20] * xx[10];
  xx[10] = xx[14] * xx[19];
  xx[11] = xx[14] * xx[20];
  xx[25] = xx[24];
  xx[26] = - xx[10];
  xx[27] = - xx[11];
  pm_math_Vector3_cross_ra(xx + 15, xx + 25, xx + 28);
  xx[14] = 0.9998343934839863;
  xx[15] = 0.0181985056119827;
  xx[16] = xx[14] * state[68] - xx[15] * state[66];
  xx[17] = xx[16] * motionData[82];
  xx[25] = xx[14] * state[69] + xx[15] * state[67];
  xx[26] = xx[25] * motionData[83];
  xx[27] = xx[17] + xx[26];
  xx[31] = xx[14] * state[66] + xx[15] * state[68];
  xx[32] = xx[14] * state[67] - xx[15] * state[69];
  xx[33] = xx[32];
  xx[34] = xx[16];
  xx[35] = xx[25];
  xx[36] = xx[32] * motionData[82];
  xx[37] = xx[32] * motionData[83];
  xx[38] = xx[27];
  xx[39] = - xx[36];
  xx[40] = - xx[37];
  pm_math_Vector3_cross_ra(xx + 33, xx + 38, xx + 41);
  xx[38] = 0.01433000000000001;
  xx[39] = xx[38] * xx[25];
  xx[40] = 0.284;
  xx[44] = xx[16] * xx[40];
  xx[45] = xx[39] - xx[44];
  xx[46] = xx[32] * xx[40];
  xx[47] = xx[32] * xx[38];
  xx[48] = xx[45];
  xx[49] = xx[46];
  xx[50] = - xx[47];
  pm_math_Vector3_cross_ra(xx + 33, xx + 48, xx + 51);
  xx[48] = (xx[31] * xx[45] + xx[51]) * xx[18];
  xx[45] = xx[16] * motionData[81];
  xx[49] = xx[32] * motionData[81];
  xx[50] = xx[26] + xx[49];
  xx[26] = xx[16] * motionData[83];
  xx[54] = - xx[45];
  xx[55] = xx[50];
  xx[56] = - xx[26];
  pm_math_Vector3_cross_ra(xx + 33, xx + 54, xx + 57);
  xx[54] = 0.5109;
  xx[55] = xx[16] * xx[54];
  xx[56] = xx[32] * xx[54];
  xx[60] = xx[39] - xx[56];
  xx[61] = xx[16] * xx[38];
  xx[62] = xx[55];
  xx[63] = xx[60];
  xx[64] = - xx[61];
  pm_math_Vector3_cross_ra(xx + 33, xx + 62, xx + 65);
  xx[62] = (xx[55] * xx[31] + xx[65]) * xx[18];
  xx[55] = xx[25] * motionData[81];
  xx[63] = xx[25] * motionData[82];
  xx[64] = xx[49] + xx[17];
  xx[68] = - xx[55];
  xx[69] = - xx[63];
  xx[70] = xx[64];
  pm_math_Vector3_cross_ra(xx + 33, xx + 68, xx + 71);
  xx[17] = xx[54] * xx[25];
  xx[49] = xx[40] * xx[25];
  xx[68] = xx[56] + xx[44];
  xx[74] = xx[17];
  xx[75] = xx[49];
  xx[76] = - xx[68];
  pm_math_Vector3_cross_ra(xx + 33, xx + 74, xx + 77);
  xx[44] = xx[18] * (xx[77] + xx[17] * xx[31]);
  xx[80] = motionData[70];
  xx[81] = motionData[71];
  xx[82] = motionData[72];
  xx[83] = motionData[73];
  xx[17] = 0.1764500000000001;
  xx[56] = xx[17] * state[75];
  xx[69] = 0.20773;
  xx[70] = xx[69] * state[76];
  xx[74] = xx[56] + xx[70];
  xx[84] = state[74];
  xx[85] = state[75];
  xx[86] = state[76];
  xx[75] = xx[17] * state[74];
  xx[87] = xx[74];
  xx[88] = - xx[75];
  xx[89] = - (xx[69] * state[74]);
  pm_math_Vector3_cross_ra(xx + 84, xx + 87, xx + 90);
  xx[87] = (xx[74] * state[73] + xx[90]) * xx[18];
  xx[88] = xx[18] * (xx[91] - xx[75] * state[73]) - xx[69];
  xx[89] = xx[17] + (xx[92] - xx[69] * state[73] * state[74]) * xx[18];
  pm_math_Quaternion_xform_ra(xx + 80, xx + 87, xx + 74);
  xx[87] = 7.5e-3;
  xx[88] = xx[87] * state[74];
  xx[89] = xx[70] + xx[88];
  xx[70] = xx[69] * state[75];
  xx[90] = - (xx[87] * state[75]);
  xx[91] = xx[89];
  xx[92] = - xx[70];
  pm_math_Vector3_cross_ra(xx + 84, xx + 90, xx + 93);
  xx[90] = xx[69] + (xx[93] - xx[87] * state[73] * state[75]) * xx[18];
  xx[91] = (xx[89] * state[73] + xx[94]) * xx[18];
  xx[92] = xx[18] * (xx[95] - xx[70] * state[73]) - xx[87];
  pm_math_Quaternion_xform_ra(xx + 80, xx + 90, xx + 93);
  xx[70] = xx[87] * state[76];
  xx[89] = xx[88] + xx[56];
  xx[90] = - xx[70];
  xx[91] = - (xx[17] * state[76]);
  xx[92] = xx[89];
  pm_math_Vector3_cross_ra(xx + 84, xx + 90, xx + 96);
  xx[84] = xx[18] * (xx[96] - xx[70] * state[73]) - xx[17];
  xx[85] = xx[87] + (xx[97] - xx[17] * state[73] * state[76]) * xx[18];
  xx[86] = (xx[89] * state[73] + xx[98]) * xx[18];
  pm_math_Quaternion_xform_ra(xx + 80, xx + 84, xx + 88);
  xx[56] = 0.1092242111199;
  xx[70] = xx[18] * (xx[52] + xx[46] * xx[31]);
  xx[46] = (xx[60] * xx[31] + xx[66]) * xx[18];
  xx[60] = (xx[49] * xx[31] + xx[78]) * xx[18];
  xx[49] = 0.02316905922206426;
  xx[51] = xx[47] * xx[31];
  xx[52] = (xx[53] - xx[51]) * xx[18];
  xx[53] = xx[61] * xx[31];
  xx[65] = xx[18] * (xx[67] - xx[53]);
  xx[66] = (xx[79] - xx[31] * xx[68]) * xx[18];
  xx[67] = xx[2] * state[36];
  xx[68] = sin(xx[67]);
  xx[77] = cos(xx[67]);
  xx[67] = xx[9] * xx[68] - xx[8] * xx[77];
  xx[78] = xx[9] * xx[77] + xx[8] * xx[68];
  xx[8] = xx[7] * xx[67] - xx[78] * xx[12];
  xx[9] = xx[1] * xx[77] + xx[5] * xx[68];
  xx[79] = xx[1] * xx[68] - xx[5] * xx[77];
  xx[84] = xx[79];
  xx[85] = - xx[78];
  xx[86] = xx[67];
  xx[1] = xx[79] * xx[12];
  xx[5] = xx[79] * xx[7];
  xx[96] = xx[8];
  xx[97] = - xx[1];
  xx[98] = - xx[5];
  pm_math_Vector3_cross_ra(xx + 84, xx + 96, xx + 99);
  xx[7] = xx[78] * xx[19] + xx[20] * xx[67];
  xx[12] = xx[79] * xx[19];
  xx[19] = xx[79] * xx[20];
  xx[77] = xx[7];
  xx[78] = xx[12];
  xx[79] = - xx[19];
  pm_math_Vector3_cross_ra(xx + 84, xx + 77, xx + 96);
  xx[20] = xx[14] * state[40] - xx[15] * state[38];
  xx[67] = xx[20] * motionData[299];
  xx[68] = xx[14] * state[41] + xx[15] * state[39];
  xx[77] = xx[68] * motionData[300];
  xx[78] = xx[67] + xx[77];
  xx[79] = xx[14] * state[38] + xx[15] * state[40];
  xx[84] = xx[14] * state[39] - xx[15] * state[41];
  xx[102] = xx[84];
  xx[103] = xx[20];
  xx[104] = xx[68];
  xx[85] = xx[84] * motionData[299];
  xx[86] = xx[84] * motionData[300];
  xx[105] = xx[78];
  xx[106] = - xx[85];
  xx[107] = - xx[86];
  pm_math_Vector3_cross_ra(xx + 102, xx + 105, xx + 108);
  xx[91] = 0.27424;
  xx[92] = xx[20] * xx[91];
  xx[105] = 8.0e-3;
  xx[106] = xx[105] * xx[68];
  xx[107] = xx[92] - xx[106];
  xx[111] = xx[84] * xx[91];
  xx[112] = xx[84] * xx[105];
  xx[113] = xx[107];
  xx[114] = - xx[111];
  xx[115] = xx[112];
  pm_math_Vector3_cross_ra(xx + 102, xx + 113, xx + 116);
  xx[113] = (xx[107] * xx[79] + xx[116]) * xx[18];
  xx[107] = xx[20] * motionData[298];
  xx[114] = xx[84] * motionData[298];
  xx[115] = xx[77] + xx[114];
  xx[77] = xx[20] * motionData[300];
  xx[119] = - xx[107];
  xx[120] = xx[115];
  xx[121] = - xx[77];
  pm_math_Vector3_cross_ra(xx + 102, xx + 119, xx + 122);
  xx[119] = 0.513;
  xx[120] = xx[20] * xx[119];
  xx[121] = xx[84] * xx[119];
  xx[125] = xx[106] + xx[121];
  xx[106] = xx[20] * xx[105];
  xx[126] = xx[120];
  xx[127] = - xx[125];
  xx[128] = xx[106];
  pm_math_Vector3_cross_ra(xx + 102, xx + 126, xx + 129);
  xx[126] = (xx[120] * xx[79] + xx[129]) * xx[18];
  xx[120] = xx[68] * motionData[298];
  xx[127] = xx[68] * motionData[299];
  xx[128] = xx[114] + xx[67];
  xx[132] = - xx[120];
  xx[133] = - xx[127];
  xx[134] = xx[128];
  pm_math_Vector3_cross_ra(xx + 102, xx + 132, xx + 135);
  xx[67] = xx[119] * xx[68];
  xx[114] = xx[91] * xx[68];
  xx[132] = xx[92] - xx[121];
  xx[138] = xx[67];
  xx[139] = - xx[114];
  xx[140] = xx[132];
  pm_math_Vector3_cross_ra(xx + 102, xx + 138, xx + 141);
  xx[92] = xx[18] * (xx[141] + xx[67] * xx[79]);
  xx[144] = motionData[42];
  xx[145] = motionData[43];
  xx[146] = motionData[44];
  xx[147] = motionData[45];
  xx[67] = 0.9999999561635808;
  xx[121] = 2.960959925766019e-4;
  xx[133] = xx[67] * state[45] - xx[121] * state[46];
  xx[134] = xx[121] * state[45] + xx[67] * state[46];
  xx[138] = xx[67] * state[47] - xx[121] * state[48];
  xx[139] = xx[67] * state[48] + xx[121] * state[47];
  xx[148] = - xx[133];
  xx[149] = xx[134];
  xx[150] = xx[138];
  xx[151] = xx[139];
  xx[140] = xx[138] * xx[138];
  xx[152] = xx[139] * xx[139];
  xx[153] = 1.0;
  xx[154] = (xx[140] + xx[152]) * xx[18] - xx[153];
  xx[155] = xx[133] * xx[139];
  xx[156] = xx[134] * xx[138];
  xx[157] = xx[155] + xx[156];
  xx[158] = xx[133] * xx[138];
  xx[159] = xx[134] * xx[139];
  xx[160] = xx[158] - xx[159];
  xx[161] = xx[154];
  xx[162] = - (xx[157] * xx[18]);
  xx[163] = xx[18] * xx[160];
  xx[164] = motionData[291];
  xx[165] = motionData[292];
  xx[166] = motionData[293];
  pm_math_Vector3_cross_ra(xx + 161, xx + 164, xx + 167);
  pm_math_Quaternion_xform_ra(xx + 148, xx + 167, xx + 170);
  xx[167] = 0.283893415005525;
  xx[168] = 0.1300676268524588;
  xx[169] = 0.1419467075027625;
  xx[173] = 0.06503381342622938;
  xx[174] = - (xx[157] * xx[167] + xx[168] * xx[160]);
  xx[175] = - (xx[154] * xx[169]);
  xx[176] = xx[154] * xx[173];
  pm_math_Quaternion_xform_ra(xx + 148, xx + 174, xx + 177);
  xx[174] = xx[170] + xx[177];
  xx[175] = xx[171] + xx[178];
  xx[176] = xx[172] + xx[179];
  pm_math_Quaternion_xform_ra(xx + 144, xx + 174, xx + 170);
  xx[154] = xx[18] * (xx[155] - xx[156]);
  xx[155] = xx[134] * xx[134];
  xx[156] = (xx[152] + xx[155]) * xx[18] - xx[153];
  xx[152] = xx[133] * xx[134];
  xx[133] = xx[139] * xx[138];
  xx[134] = xx[152] + xx[133];
  xx[174] = xx[154];
  xx[175] = xx[156];
  xx[176] = - (xx[134] * xx[18]);
  pm_math_Vector3_cross_ra(xx + 174, xx + 164, xx + 180);
  pm_math_Quaternion_xform_ra(xx + 148, xx + 180, xx + 183);
  xx[180] = xx[156] * xx[169] + xx[134] * xx[168];
  xx[181] = - (xx[169] * xx[154]);
  xx[182] = xx[173] * xx[154];
  pm_math_Quaternion_xform_ra(xx + 148, xx + 180, xx + 186);
  xx[180] = xx[183] + xx[186];
  xx[181] = xx[184] + xx[187];
  xx[182] = xx[185] + xx[188];
  pm_math_Quaternion_xform_ra(xx + 144, xx + 180, xx + 183);
  xx[134] = (xx[158] + xx[159]) * xx[18];
  xx[138] = xx[152] - xx[133];
  xx[133] = (xx[155] + xx[140]) * xx[18] - xx[153];
  xx[154] = - xx[134];
  xx[155] = xx[18] * xx[138];
  xx[156] = xx[133];
  pm_math_Vector3_cross_ra(xx + 154, xx + 164, xx + 157);
  pm_math_Quaternion_xform_ra(xx + 148, xx + 157, xx + 164);
  xx[157] = xx[167] * xx[138] - xx[133] * xx[173];
  xx[158] = xx[169] * xx[134];
  xx[159] = - (xx[173] * xx[134]);
  pm_math_Quaternion_xform_ra(xx + 148, xx + 157, xx + 138);
  xx[157] = xx[164] + xx[138];
  xx[158] = xx[165] + xx[139];
  xx[159] = xx[166] + xx[140];
  pm_math_Quaternion_xform_ra(xx + 144, xx + 157, xx + 164);
  xx[157] = motionData[210];
  xx[158] = motionData[211];
  xx[159] = motionData[212];
  xx[160] = motionData[213];
  xx[133] = xx[67] * state[52] + xx[121] * state[53];
  xx[134] = xx[121] * state[52] - xx[67] * state[53];
  xx[152] = xx[121] * state[55] - xx[67] * state[54];
  xx[167] = xx[67] * state[55] + xx[121] * state[54];
  xx[121] = - xx[167];
  xx[189] = xx[133];
  xx[190] = xx[134];
  xx[191] = xx[152];
  xx[192] = xx[121];
  xx[180] = (xx[152] * xx[152] + xx[167] * xx[167]) * xx[18] - xx[153];
  xx[181] = - ((xx[133] * xx[167] + xx[134] * xx[152]) * xx[18]);
  xx[182] = xx[18] * (xx[167] * xx[134] - xx[133] * xx[152]);
  xx[193] = motionData[67];
  xx[194] = motionData[68];
  xx[195] = motionData[69];
  pm_math_Vector3_cross_ra(xx + 180, xx + 193, xx + 196);
  pm_math_Quaternion_xform_ra(xx + 189, xx + 196, xx + 199);
  pm_math_Quaternion_xform_ra(xx + 157, xx + 199, xx + 196);
  xx[199] = xx[134];
  xx[200] = xx[152];
  xx[201] = xx[121];
  xx[121] = 5.921919591936278e-4;
  xx[168] = 0.9999998246543264;
  xx[169] = xx[121] * xx[152] + xx[167] * xx[168];
  xx[173] = xx[121] * xx[134];
  xx[202] = xx[168] * xx[134];
  xx[203] = - xx[169];
  xx[204] = xx[173];
  xx[205] = - xx[202];
  pm_math_Vector3_cross_ra(xx + 199, xx + 203, xx + 206);
  xx[134] = xx[133] * xx[173];
  xx[203] = xx[133] * xx[202];
  xx[209] = xx[18] * (xx[206] + xx[133] * xx[169]);
  xx[210] = (xx[207] - xx[134]) * xx[18] - xx[168];
  xx[211] = xx[18] * (xx[208] + xx[203]) - xx[121];
  pm_math_Vector3_cross_ra(xx + 209, xx + 193, xx + 204);
  pm_math_Quaternion_xform_ra(xx + 189, xx + 204, xx + 212);
  pm_math_Quaternion_xform_ra(xx + 157, xx + 212, xx + 204);
  xx[169] = xx[167] * xx[121] - xx[168] * xx[152];
  xx[212] = xx[169];
  xx[213] = xx[202];
  xx[214] = xx[173];
  pm_math_Vector3_cross_ra(xx + 199, xx + 212, xx + 215);
  xx[199] = xx[18] * (xx[215] - xx[133] * xx[169]);
  xx[200] = xx[121] + (xx[216] - xx[203]) * xx[18];
  xx[201] = xx[18] * (xx[217] - xx[134]) - xx[168];
  pm_math_Vector3_cross_ra(xx + 199, xx + 193, xx + 212);
  pm_math_Quaternion_xform_ra(xx + 189, xx + 212, xx + 193);
  pm_math_Quaternion_xform_ra(xx + 157, xx + 193, xx + 212);
  xx[215] = motionData[273];
  xx[216] = motionData[274];
  xx[217] = motionData[275];
  xx[218] = motionData[276];
  xx[121] = xx[69] * state[62];
  xx[133] = xx[17] * state[61];
  xx[134] = xx[121] - xx[133];
  xx[193] = state[60];
  xx[194] = state[61];
  xx[195] = state[62];
  xx[152] = xx[17] * state[60];
  xx[219] = xx[134];
  xx[220] = xx[152];
  xx[221] = - (xx[69] * state[60]);
  pm_math_Vector3_cross_ra(xx + 193, xx + 219, xx + 222);
  xx[219] = (xx[134] * state[59] + xx[222]) * xx[18];
  xx[220] = xx[18] * (xx[223] + xx[152] * state[59]) - xx[69];
  xx[221] = (xx[224] - xx[69] * state[59] * state[60]) * xx[18] - xx[17];
  pm_math_Quaternion_xform_ra(xx + 215, xx + 219, xx + 222);
  xx[134] = xx[87] * state[60];
  xx[152] = xx[121] + xx[134];
  xx[121] = xx[69] * state[61];
  xx[219] = - (xx[87] * state[61]);
  xx[220] = xx[152];
  xx[221] = - xx[121];
  pm_math_Vector3_cross_ra(xx + 193, xx + 219, xx + 225);
  xx[219] = xx[69] + (xx[225] - xx[87] * state[59] * state[61]) * xx[18];
  xx[220] = (xx[152] * state[59] + xx[226]) * xx[18];
  xx[221] = xx[18] * (xx[227] - xx[121] * state[59]) - xx[87];
  pm_math_Quaternion_xform_ra(xx + 215, xx + 219, xx + 225);
  xx[69] = xx[87] * state[62];
  xx[121] = xx[134] - xx[133];
  xx[219] = - xx[69];
  xx[220] = xx[17] * state[62];
  xx[221] = xx[121];
  pm_math_Vector3_cross_ra(xx + 193, xx + 219, xx + 228);
  xx[193] = xx[17] + xx[18] * (xx[228] - xx[69] * state[59]);
  xx[194] = xx[87] + (xx[17] * state[59] * state[62] + xx[229]) * xx[18];
  xx[195] = (xx[121] * state[59] + xx[230]) * xx[18];
  pm_math_Quaternion_xform_ra(xx + 215, xx + 193, xx + 219);
  xx[17] = xx[18] * (xx[117] - xx[111] * xx[79]);
  xx[69] = (xx[130] - xx[79] * xx[125]) * xx[18];
  xx[87] = (xx[142] - xx[114] * xx[79]) * xx[18];
  xx[111] = (xx[112] * xx[79] + xx[118]) * xx[18];
  xx[112] = xx[18] * (xx[131] + xx[106] * xx[79]);
  xx[106] = (xx[79] * xx[132] + xx[143]) * xx[18];
  xx[129] = xx[31];
  xx[130] = xx[32];
  xx[131] = xx[16];
  xx[132] = xx[25];
  xx[215] = motionData[308];
  xx[216] = motionData[309];
  xx[217] = motionData[310];
  xx[218] = motionData[311];
  pm_math_Quaternion_compose_ra(xx + 129, xx + 215, xx + 228);
  xx[114] = motionData[310] * motionData[310];
  xx[116] = motionData[311] * motionData[311];
  xx[117] = motionData[309] * motionData[310];
  xx[118] = motionData[308] * motionData[311];
  xx[121] = motionData[308] * motionData[310];
  xx[125] = motionData[309] * motionData[311];
  xx[129] = xx[153] - (xx[114] + xx[116]) * xx[18];
  xx[130] = xx[18] * (xx[117] - xx[118]);
  xx[131] = (xx[121] + xx[125]) * xx[18];
  xx[132] = xx[119];
  xx[133] = xx[91];
  xx[134] = xx[105];
  pm_math_Vector3_cross_ra(xx + 129, xx + 132, xx + 141);
  pm_math_Quaternion_xform_ra(xx + 228, xx + 141, xx + 193);
  xx[141] = xx[16] * motionData[313];
  xx[142] = xx[25] * motionData[314];
  xx[143] = xx[141] + xx[142];
  xx[152] = xx[32] * motionData[313];
  xx[167] = xx[32] * motionData[314];
  xx[215] = xx[143];
  xx[216] = - xx[152];
  xx[217] = - xx[167];
  pm_math_Vector3_cross_ra(xx + 33, xx + 215, xx + 232);
  xx[169] = (xx[143] * xx[31] + xx[232]) * xx[18] + xx[48];
  xx[143] = motionData[309] * motionData[309];
  xx[173] = motionData[310] * motionData[311];
  xx[202] = motionData[308] * motionData[309];
  xx[215] = (xx[118] + xx[117]) * xx[18];
  xx[216] = xx[153] - (xx[116] + xx[143]) * xx[18];
  xx[217] = xx[18] * (xx[173] - xx[202]);
  pm_math_Vector3_cross_ra(xx + 215, xx + 132, xx + 116);
  pm_math_Quaternion_xform_ra(xx + 228, xx + 116, xx + 235);
  xx[116] = xx[16] * motionData[312];
  xx[117] = xx[32] * motionData[312];
  xx[118] = xx[142] + xx[117];
  xx[142] = xx[16] * motionData[314];
  xx[238] = - xx[116];
  xx[239] = xx[118];
  xx[240] = - xx[142];
  pm_math_Vector3_cross_ra(xx + 33, xx + 238, xx + 241);
  xx[203] = motionData[314] + xx[18] * (xx[241] - xx[116] * xx[31]) + xx[62];
  xx[238] = xx[18] * (xx[125] - xx[121]);
  xx[239] = (xx[202] + xx[173]) * xx[18];
  xx[240] = xx[153] - (xx[143] + xx[114]) * xx[18];
  pm_math_Vector3_cross_ra(xx + 238, xx + 132, xx + 244);
  pm_math_Quaternion_xform_ra(xx + 228, xx + 244, xx + 247);
  xx[114] = xx[25] * motionData[312];
  xx[116] = xx[25] * motionData[313];
  xx[121] = xx[117] + xx[141];
  xx[244] = - xx[114];
  xx[245] = - xx[116];
  xx[246] = xx[121];
  pm_math_Vector3_cross_ra(xx + 33, xx + 244, xx + 250);
  xx[117] = xx[18] * (xx[250] - xx[114] * xx[31]) + xx[44] - motionData[313];
  xx[114] = 2.960959925763799e-4;
  xx[125] = xx[114] * state[81] - xx[67] * state[80];
  xx[141] = xx[114] * state[80] + xx[67] * state[81];
  xx[143] = - xx[141];
  xx[173] = xx[114] * state[83] + xx[67] * state[82];
  xx[202] = - xx[173];
  xx[207] = xx[67] * state[83] - xx[114] * state[82];
  xx[208] = - xx[207];
  xx[253] = xx[125];
  xx[254] = xx[143];
  xx[255] = xx[202];
  xx[256] = xx[208];
  pm_math_Quaternion_compose_ra(xx + 80, xx + 253, xx + 257);
  xx[261] = motionData[91];
  xx[262] = motionData[92];
  xx[263] = motionData[93];
  xx[264] = motionData[94];
  pm_math_Quaternion_compose_ra(xx + 257, xx + 261, xx + 265);
  xx[244] = xx[153] - (motionData[93] * motionData[93] + motionData[94] *
                       motionData[94]) * xx[18];
  xx[245] = xx[18] * (motionData[92] * motionData[93] - motionData[91] *
                      motionData[94]);
  xx[246] = (motionData[91] * motionData[93] + motionData[92] * motionData[94]) *
    xx[18];
  pm_math_Vector3_cross_ra(xx + 244, xx + 132, xx + 257);
  pm_math_Quaternion_xform_ra(xx + 265, xx + 257, xx + 260);
  xx[257] = xx[143];
  xx[258] = xx[202];
  xx[259] = xx[208];
  xx[143] = xx[173] * motionData[96] + xx[207] * motionData[97];
  xx[202] = xx[141] * motionData[96];
  xx[208] = xx[141] * motionData[97];
  xx[269] = - xx[143];
  xx[270] = xx[202];
  xx[271] = xx[208];
  pm_math_Vector3_cross_ra(xx + 257, xx + 269, xx + 272);
  xx[218] = 0.09037053171319967;
  xx[263] = 0.0213683169329786;
  xx[264] = xx[218] * xx[173] + xx[207] * xx[263];
  xx[269] = xx[218] * xx[141];
  xx[270] = xx[263] * xx[141];
  xx[275] = xx[264];
  xx[276] = - xx[269];
  xx[277] = - xx[270];
  pm_math_Vector3_cross_ra(xx + 257, xx + 275, xx + 278);
  xx[257] = (xx[272] - xx[143] * xx[125]) * xx[18] + (xx[264] * xx[125] + xx[278])
    * xx[18];
  xx[258] = xx[18] * (xx[273] + xx[202] * xx[125]) + xx[18] * (xx[279] - xx[269]
    * xx[125]) - motionData[97] + xx[263];
  xx[259] = motionData[96] + xx[18] * (xx[274] + xx[208] * xx[125]) + (xx[280] -
    xx[270] * xx[125]) * xx[18] - xx[218];
  pm_math_Quaternion_xform_ra(xx + 80, xx + 257, xx + 269);
  xx[257] = motionData[92];
  xx[258] = motionData[93];
  xx[259] = motionData[94];
  xx[143] = 5.921919591931837e-4;
  xx[202] = xx[143] * motionData[93] + xx[168] * motionData[94];
  xx[208] = xx[143] * motionData[92];
  xx[218] = xx[168] * motionData[92];
  xx[272] = - xx[202];
  xx[273] = xx[208];
  xx[274] = xx[218];
  pm_math_Vector3_cross_ra(xx + 257, xx + 272, xx + 275);
  xx[263] = motionData[91] * motionData[92];
  xx[272] = xx[18] * (xx[275] + xx[202] * motionData[91]);
  xx[273] = xx[168] + (xx[276] - xx[208] * motionData[91]) * xx[18];
  xx[274] = xx[18] * (xx[277] - xx[168] * xx[263]) - xx[143];
  pm_math_Vector3_cross_ra(xx + 272, xx + 132, xx + 275);
  pm_math_Quaternion_xform_ra(xx + 265, xx + 275, xx + 278);
  xx[202] = - (xx[143] * motionData[95]);
  xx[264] = xx[168] * motionData[95];
  xx[275] = xx[168] * motionData[97] + xx[143] * motionData[96];
  xx[276] = xx[202];
  xx[277] = - xx[264];
  pm_math_Quaternion_xform_ra(xx + 253, xx + 275, xx + 281);
  xx[275] = 0.02142182988836524;
  xx[276] = xx[275] * xx[173];
  xx[277] = xx[207] * xx[275];
  xx[284] = xx[281] + (xx[276] * xx[173] + xx[207] * xx[277]) * xx[18] - xx[275];
  xx[285] = xx[282] + xx[18] * (xx[277] * xx[125] - xx[276] * xx[141]);
  xx[286] = xx[283] - (xx[276] * xx[125] + xx[277] * xx[141]) * xx[18];
  pm_math_Quaternion_xform_ra(xx + 80, xx + 284, xx + 275);
  xx[281] = xx[168] * motionData[93] - xx[143] * motionData[94];
  xx[282] = xx[281];
  xx[283] = - xx[218];
  xx[284] = xx[208];
  pm_math_Vector3_cross_ra(xx + 257, xx + 282, xx + 285);
  xx[257] = xx[18] * (xx[285] - xx[281] * motionData[91]);
  xx[258] = xx[143] + (xx[218] * motionData[91] + xx[286]) * xx[18];
  xx[259] = xx[168] + xx[18] * (xx[287] - xx[143] * xx[263]);
  pm_math_Vector3_cross_ra(xx + 257, xx + 132, xx + 281);
  pm_math_Quaternion_xform_ra(xx + 265, xx + 281, xx + 132);
  xx[281] = xx[143] * motionData[97] - xx[168] * motionData[96];
  xx[282] = xx[264];
  xx[283] = xx[202];
  pm_math_Quaternion_xform_ra(xx + 253, xx + 281, xx + 284);
  xx[143] = 0.0903578617216487;
  xx[168] = xx[143] * xx[173];
  xx[202] = xx[207] * xx[143];
  xx[253] = xx[284] - (xx[168] * xx[173] + xx[207] * xx[202]) * xx[18] + xx[143];
  xx[254] = xx[285] + xx[18] * (xx[168] * xx[141] - xx[202] * xx[125]);
  xx[255] = xx[286] + (xx[168] * xx[125] + xx[202] * xx[141]) * xx[18];
  pm_math_Quaternion_xform_ra(xx + 80, xx + 253, xx + 281);
  xx[80] = motionData[175];
  xx[81] = motionData[176];
  xx[82] = motionData[177];
  xx[83] = motionData[178];
  xx[253] = xx[114] * state[88] + xx[67] * state[87];
  xx[254] = - (xx[114] * state[87] - xx[67] * state[88]);
  xx[255] = xx[114] * state[90] + xx[67] * state[89];
  xx[256] = - (xx[114] * state[89] - xx[67] * state[90]);
  pm_math_Quaternion_compose_ra(xx + 80, xx + 253, xx + 284);
  xx[67] = xx[91] * xx[286];
  xx[80] = xx[105] * xx[287];
  xx[81] = xx[67] + xx[80];
  xx[82] = xx[91] * xx[285];
  xx[253] = xx[81];
  xx[254] = - xx[82];
  xx[255] = - (xx[105] * xx[285]);
  pm_math_Vector3_cross_ra(xx + 285, xx + 253, xx + 288);
  xx[83] = xx[119] * xx[285];
  xx[114] = xx[80] + xx[83];
  xx[80] = xx[105] * xx[286];
  xx[253] = - (xx[119] * xx[286]);
  xx[254] = xx[114];
  xx[255] = - xx[80];
  pm_math_Vector3_cross_ra(xx + 285, xx + 253, xx + 291);
  xx[125] = xx[284] * xx[286];
  xx[141] = xx[119] * xx[287];
  xx[143] = xx[83] + xx[67];
  xx[253] = - xx[141];
  xx[254] = - (xx[91] * xx[287]);
  xx[255] = xx[143];
  pm_math_Vector3_cross_ra(xx + 285, xx + 253, xx + 294);
  xx[67] = xx[18] * (xx[233] - xx[152] * xx[31]) + xx[70] - motionData[314];
  xx[83] = (xx[118] * xx[31] + xx[242]) * xx[18] + xx[46];
  xx[118] = motionData[312] + xx[18] * (xx[251] - xx[116] * xx[31]) + xx[60];
  xx[116] = xx[284] * xx[287];
  xx[152] = motionData[313] + xx[18] * (xx[234] - xx[167] * xx[31]) + xx[52];
  xx[167] = xx[18] * (xx[243] - xx[142] * xx[31]) + xx[65] - motionData[312];
  xx[142] = (xx[121] * xx[31] + xx[252]) * xx[18] + xx[66];
  xx[121] = xx[284] * xx[285];
  xx[168] = 8.94e-3;
  xx[173] = 0.3143400000000001;
  xx[202] = 9.000000000000001e-3;
  xx[232] = - xx[168];
  xx[233] = xx[173];
  xx[234] = xx[202];
  pm_math_Vector3_cross_ra(xx + 129, xx + 232, xx + 241);
  pm_math_Quaternion_xform_ra(xx + 228, xx + 241, xx + 129);
  pm_math_Vector3_cross_ra(xx + 215, xx + 232, xx + 241);
  pm_math_Quaternion_xform_ra(xx + 228, xx + 241, xx + 215);
  pm_math_Vector3_cross_ra(xx + 238, xx + 232, xx + 241);
  pm_math_Quaternion_xform_ra(xx + 228, xx + 241, xx + 238);
  pm_math_Vector3_cross_ra(xx + 244, xx + 232, xx + 228);
  pm_math_Quaternion_xform_ra(xx + 265, xx + 228, xx + 241);
  pm_math_Vector3_cross_ra(xx + 272, xx + 232, xx + 228);
  pm_math_Quaternion_xform_ra(xx + 265, xx + 228, xx + 244);
  pm_math_Vector3_cross_ra(xx + 257, xx + 232, xx + 228);
  pm_math_Quaternion_xform_ra(xx + 265, xx + 228, xx + 231);
  xx[207] = xx[173] * xx[286];
  xx[208] = xx[202] * xx[287];
  xx[218] = xx[207] + xx[208];
  xx[228] = xx[173] * xx[285];
  xx[250] = xx[218];
  xx[251] = - xx[228];
  xx[252] = - (xx[202] * xx[285]);
  pm_math_Vector3_cross_ra(xx + 285, xx + 250, xx + 253);
  xx[229] = xx[168] * xx[285];
  xx[230] = xx[208] - xx[229];
  xx[208] = xx[202] * xx[286];
  xx[250] = xx[168] * xx[286];
  xx[251] = xx[230];
  xx[252] = - xx[208];
  pm_math_Vector3_cross_ra(xx + 285, xx + 250, xx + 256);
  xx[234] = xx[168] * xx[287];
  xx[250] = xx[207] - xx[229];
  xx[263] = xx[234];
  xx[264] = - (xx[173] * xx[287]);
  xx[265] = xx[250];
  pm_math_Vector3_cross_ra(xx + 285, xx + 263, xx + 266);
  xx[207] = xx[202] * xx[68];
  xx[229] = xx[20] * xx[173];
  xx[251] = xx[207] - xx[229];
  xx[252] = xx[84] * xx[173];
  xx[259] = xx[84] * xx[202];
  xx[263] = xx[251];
  xx[264] = xx[252];
  xx[265] = - xx[259];
  pm_math_Vector3_cross_ra(xx + 102, xx + 263, xx + 272);
  xx[263] = xx[20] * xx[168];
  xx[264] = xx[84] * xx[168];
  xx[265] = xx[207] - xx[264];
  xx[207] = xx[20] * xx[202];
  xx[285] = xx[263];
  xx[286] = xx[265];
  xx[287] = - xx[207];
  pm_math_Vector3_cross_ra(xx + 102, xx + 285, xx + 297);
  xx[285] = 1.000000000000001e-3;
  xx[286] = xx[168] * xx[68];
  xx[287] = xx[173] * xx[68];
  xx[300] = xx[264] + xx[229];
  xx[301] = xx[286];
  xx[302] = xx[287];
  xx[303] = - xx[300];
  pm_math_Vector3_cross_ra(xx + 102, xx + 301, xx + 304);
  xx[229] = 0.04010000000000002;
  xx[264] = 0.52194;
  xx[307] = xx[79];
  xx[308] = xx[84];
  xx[309] = xx[20];
  xx[310] = xx[68];
  xx[311] = motionData[315];
  xx[312] = motionData[316];
  xx[313] = motionData[317];
  xx[314] = motionData[318];
  pm_math_Quaternion_compose_ra(xx + 307, xx + 311, xx + 315);
  xx[301] = motionData[317] * motionData[317];
  xx[302] = motionData[318] * motionData[318];
  xx[303] = motionData[316] * motionData[317];
  xx[307] = motionData[315] * motionData[318];
  xx[308] = motionData[315] * motionData[317];
  xx[309] = motionData[316] * motionData[318];
  xx[310] = xx[153] - (xx[301] + xx[302]) * xx[18];
  xx[311] = xx[18] * (xx[303] - xx[307]);
  xx[312] = (xx[308] + xx[309]) * xx[18];
  xx[313] = - xx[38];
  xx[319] = xx[54];
  xx[320] = - xx[40];
  xx[321] = xx[313];
  pm_math_Vector3_cross_ra(xx + 310, xx + 319, xx + 322);
  pm_math_Quaternion_xform_ra(xx + 315, xx + 322, xx + 325);
  xx[314] = xx[20] * motionData[320];
  xx[322] = xx[68] * motionData[321];
  xx[323] = xx[314] + xx[322];
  xx[324] = xx[84] * motionData[320];
  xx[328] = xx[84] * motionData[321];
  xx[329] = xx[323];
  xx[330] = - xx[324];
  xx[331] = - xx[328];
  pm_math_Vector3_cross_ra(xx + 102, xx + 329, xx + 332);
  xx[329] = (xx[323] * xx[79] + xx[332]) * xx[18] + xx[113];
  xx[323] = motionData[316] * motionData[316];
  xx[330] = motionData[317] * motionData[318];
  xx[331] = motionData[315] * motionData[316];
  xx[335] = (xx[307] + xx[303]) * xx[18];
  xx[336] = xx[153] - (xx[302] + xx[323]) * xx[18];
  xx[337] = xx[18] * (xx[330] - xx[331]);
  pm_math_Vector3_cross_ra(xx + 335, xx + 319, xx + 338);
  pm_math_Quaternion_xform_ra(xx + 315, xx + 338, xx + 341);
  xx[302] = xx[20] * motionData[319];
  xx[303] = xx[84] * motionData[319];
  xx[84] = xx[322] + xx[303];
  xx[307] = xx[20] * motionData[321];
  xx[338] = - xx[302];
  xx[339] = xx[84];
  xx[340] = - xx[307];
  pm_math_Vector3_cross_ra(xx + 102, xx + 338, xx + 344);
  xx[20] = motionData[321] + xx[18] * (xx[344] - xx[302] * xx[79]) + xx[126];
  xx[338] = xx[18] * (xx[309] - xx[308]);
  xx[339] = (xx[331] + xx[330]) * xx[18];
  xx[340] = xx[153] - (xx[323] + xx[301]) * xx[18];
  pm_math_Vector3_cross_ra(xx + 338, xx + 319, xx + 347);
  pm_math_Quaternion_xform_ra(xx + 315, xx + 347, xx + 350);
  xx[301] = xx[68] * motionData[319];
  xx[302] = xx[68] * motionData[320];
  xx[68] = xx[303] + xx[314];
  xx[347] = - xx[301];
  xx[348] = - xx[302];
  xx[349] = xx[68];
  pm_math_Vector3_cross_ra(xx + 102, xx + 347, xx + 353);
  xx[102] = xx[18] * (xx[353] - xx[301] * xx[79]) + xx[92] - motionData[320];
  pm_math_Quaternion_compose_ra(xx + 144, xx + 148, xx + 356);
  xx[360] = motionData[56];
  xx[361] = motionData[57];
  xx[362] = motionData[58];
  xx[363] = motionData[59];
  pm_math_Quaternion_compose_ra(xx + 356, xx + 360, xx + 364);
  pm_math_Quaternion_inverseXform_ra(xx + 360, xx + 161, xx + 347);
  pm_math_Vector3_cross_ra(xx + 347, xx + 319, xx + 356);
  pm_math_Quaternion_xform_ra(xx + 364, xx + 356, xx + 368);
  xx[356] = motionData[60];
  xx[357] = motionData[61];
  xx[358] = motionData[62];
  pm_math_Vector3_cross_ra(xx + 161, xx + 356, xx + 371);
  pm_math_Quaternion_xform_ra(xx + 148, xx + 371, xx + 161);
  xx[371] = xx[161] + xx[177];
  xx[372] = xx[162] + xx[178];
  xx[373] = xx[163] + xx[179];
  pm_math_Quaternion_xform_ra(xx + 144, xx + 371, xx + 161);
  pm_math_Quaternion_inverseXform_ra(xx + 360, xx + 174, xx + 177);
  pm_math_Vector3_cross_ra(xx + 177, xx + 319, xx + 371);
  pm_math_Quaternion_xform_ra(xx + 364, xx + 371, xx + 374);
  pm_math_Vector3_cross_ra(xx + 174, xx + 356, xx + 371);
  pm_math_Quaternion_xform_ra(xx + 148, xx + 371, xx + 174);
  xx[371] = xx[174] + xx[186];
  xx[372] = xx[175] + xx[187];
  xx[373] = xx[176] + xx[188];
  pm_math_Quaternion_xform_ra(xx + 144, xx + 371, xx + 174);
  pm_math_Quaternion_inverseXform_ra(xx + 360, xx + 154, xx + 186);
  pm_math_Vector3_cross_ra(xx + 186, xx + 319, xx + 359);
  pm_math_Quaternion_xform_ra(xx + 364, xx + 359, xx + 371);
  pm_math_Vector3_cross_ra(xx + 154, xx + 356, xx + 359);
  pm_math_Quaternion_xform_ra(xx + 148, xx + 359, xx + 154);
  xx[148] = xx[154] + xx[138];
  xx[149] = xx[155] + xx[139];
  xx[150] = xx[156] + xx[140];
  pm_math_Quaternion_xform_ra(xx + 144, xx + 148, xx + 138);
  pm_math_Quaternion_compose_ra(xx + 157, xx + 189, xx + 144);
  pm_math_Vector3_cross_ra(xx + 180, xx + 319, xx + 148);
  pm_math_Quaternion_xform_ra(xx + 144, xx + 148, xx + 154);
  pm_math_Vector3_cross_ra(xx + 209, xx + 319, xx + 148);
  pm_math_Quaternion_xform_ra(xx + 144, xx + 148, xx + 157);
  pm_math_Vector3_cross_ra(xx + 199, xx + 319, xx + 148);
  pm_math_Quaternion_xform_ra(xx + 144, xx + 148, xx + 189);
  xx[103] = xx[18] * (xx[333] - xx[324] * xx[79]) + xx[17] - motionData[321];
  xx[104] = (xx[84] * xx[79] + xx[345]) * xx[18] + xx[69];
  xx[84] = motionData[319] + xx[18] * (xx[354] - xx[302] * xx[79]) + xx[87];
  xx[148] = motionData[320] + xx[18] * (xx[334] - xx[328] * xx[79]) + xx[111];
  xx[149] = xx[18] * (xx[346] - xx[307] * xx[79]) + xx[112] - motionData[319];
  xx[150] = (xx[68] * xx[79] + xx[355]) * xx[18] + xx[106];
  xx[68] = 8.82e-3;
  xx[151] = 0.2541;
  xx[301] = - xx[68];
  xx[302] = - xx[151];
  xx[303] = xx[313];
  pm_math_Vector3_cross_ra(xx + 310, xx + 301, xx + 307);
  pm_math_Quaternion_xform_ra(xx + 315, xx + 307, xx + 310);
  pm_math_Vector3_cross_ra(xx + 335, xx + 301, xx + 307);
  pm_math_Quaternion_xform_ra(xx + 315, xx + 307, xx + 319);
  pm_math_Vector3_cross_ra(xx + 338, xx + 301, xx + 307);
  pm_math_Quaternion_xform_ra(xx + 315, xx + 307, xx + 322);
  pm_math_Vector3_cross_ra(xx + 347, xx + 301, xx + 307);
  pm_math_Quaternion_xform_ra(xx + 364, xx + 307, xx + 313);
  pm_math_Vector3_cross_ra(xx + 177, xx + 301, xx + 307);
  pm_math_Quaternion_xform_ra(xx + 364, xx + 307, xx + 177);
  pm_math_Vector3_cross_ra(xx + 186, xx + 301, xx + 307);
  pm_math_Quaternion_xform_ra(xx + 364, xx + 307, xx + 186);
  pm_math_Vector3_cross_ra(xx + 180, xx + 301, xx + 307);
  pm_math_Quaternion_xform_ra(xx + 144, xx + 307, xx + 180);
  pm_math_Vector3_cross_ra(xx + 209, xx + 301, xx + 307);
  pm_math_Quaternion_xform_ra(xx + 144, xx + 307, xx + 209);
  pm_math_Vector3_cross_ra(xx + 199, xx + 301, xx + 307);
  pm_math_Quaternion_xform_ra(xx + 144, xx + 307, xx + 199);
  xx[144] = xx[16] * xx[151];
  xx[145] = xx[144] - xx[39];
  xx[146] = xx[32] * xx[151];
  xx[301] = xx[145];
  xx[302] = - xx[146];
  xx[303] = xx[47];
  pm_math_Vector3_cross_ra(xx + 33, xx + 301, xx + 307);
  xx[47] = xx[16] * xx[68];
  xx[16] = xx[32] * xx[68];
  xx[32] = xx[39] + xx[16];
  xx[301] = xx[47];
  xx[302] = - xx[32];
  xx[303] = xx[61];
  pm_math_Vector3_cross_ra(xx + 33, xx + 301, xx + 316);
  xx[39] = xx[68] * xx[25];
  xx[61] = xx[151] * xx[25];
  xx[25] = xx[144] - xx[16];
  xx[301] = xx[39];
  xx[302] = - xx[61];
  xx[303] = xx[25];
  pm_math_Vector3_cross_ra(xx + 33, xx + 301, xx + 330);
  xx[16] = 0.02989999999999998;
  xx[33] = 0.5197200000000001;
  xx[34] = xx[14] * state[98] + xx[15] * state[100];
  xx[35] = xx[14] * state[99] - xx[15] * state[101];
  xx[68] = xx[14] * state[100] - xx[15] * state[98];
  xx[144] = xx[14] * state[101] + xx[15] * state[99];
  xx[333] = xx[34];
  xx[334] = xx[35];
  xx[335] = xx[68];
  xx[336] = xx[144];
  xx[337] = motionData[329];
  xx[338] = motionData[330];
  xx[339] = motionData[331];
  xx[340] = motionData[332];
  pm_math_Quaternion_compose_ra(xx + 333, xx + 337, xx + 344);
  xx[147] = motionData[331] * motionData[331];
  xx[151] = motionData[332] * motionData[332];
  xx[160] = motionData[330] * motionData[331];
  xx[192] = motionData[329] * motionData[332];
  xx[301] = motionData[329] * motionData[331];
  xx[302] = motionData[330] * motionData[332];
  xx[333] = xx[153] - (xx[147] + xx[151]) * xx[18];
  xx[334] = xx[18] * (xx[160] - xx[192]);
  xx[335] = (xx[301] + xx[302]) * xx[18];
  xx[303] = 0.1378799999999999;
  xx[328] = 0.29899;
  xx[336] = 6.500000000000001e-3;
  xx[337] = xx[303];
  xx[338] = xx[328];
  xx[339] = - xx[336];
  pm_math_Vector3_cross_ra(xx + 333, xx + 337, xx + 353);
  pm_math_Quaternion_xform_ra(xx + 344, xx + 353, xx + 356);
  xx[340] = xx[68] * motionData[334];
  xx[348] = xx[144] * motionData[335];
  xx[349] = xx[340] + xx[348];
  xx[353] = xx[35];
  xx[354] = xx[68];
  xx[355] = xx[144];
  xx[359] = xx[35] * motionData[334];
  xx[360] = xx[35] * motionData[335];
  xx[361] = xx[349];
  xx[362] = - xx[359];
  xx[363] = - xx[360];
  pm_math_Vector3_cross_ra(xx + 353, xx + 361, xx + 364);
  xx[361] = 0.05344999999999999;
  xx[362] = xx[361] * xx[144];
  xx[363] = 0.26683;
  xx[367] = xx[68] * xx[363];
  xx[377] = xx[362] - xx[367];
  xx[378] = xx[35] * xx[363];
  xx[379] = xx[35] * xx[361];
  xx[380] = xx[377];
  xx[381] = xx[378];
  xx[382] = - xx[379];
  pm_math_Vector3_cross_ra(xx + 353, xx + 380, xx + 383);
  xx[380] = (xx[34] * xx[377] + xx[383]) * xx[18];
  xx[377] = (xx[349] * xx[34] + xx[364]) * xx[18] + xx[380];
  xx[349] = motionData[330] * motionData[330];
  xx[381] = motionData[331] * motionData[332];
  xx[382] = motionData[329] * motionData[330];
  xx[386] = (xx[192] + xx[160]) * xx[18];
  xx[387] = xx[153] - (xx[151] + xx[349]) * xx[18];
  xx[388] = xx[18] * (xx[381] - xx[382]);
  pm_math_Vector3_cross_ra(xx + 386, xx + 337, xx + 389);
  pm_math_Quaternion_xform_ra(xx + 344, xx + 389, xx + 392);
  xx[151] = xx[68] * motionData[333];
  xx[160] = xx[35] * motionData[333];
  xx[192] = xx[348] + xx[160];
  xx[348] = xx[68] * motionData[335];
  xx[389] = - xx[151];
  xx[390] = xx[192];
  xx[391] = - xx[348];
  pm_math_Vector3_cross_ra(xx + 353, xx + 389, xx + 395);
  xx[389] = 0.14279;
  xx[390] = xx[68] * xx[389];
  xx[391] = xx[35] * xx[389];
  xx[398] = xx[362] - xx[391];
  xx[362] = xx[68] * xx[361];
  xx[399] = xx[390];
  xx[400] = xx[398];
  xx[401] = - xx[362];
  pm_math_Vector3_cross_ra(xx + 353, xx + 399, xx + 402);
  xx[399] = (xx[390] * xx[34] + xx[402]) * xx[18];
  xx[390] = motionData[335] + xx[18] * (xx[395] - xx[151] * xx[34]) + xx[399];
  xx[405] = xx[18] * (xx[302] - xx[301]);
  xx[406] = (xx[382] + xx[381]) * xx[18];
  xx[407] = xx[153] - (xx[349] + xx[147]) * xx[18];
  pm_math_Vector3_cross_ra(xx + 405, xx + 337, xx + 408);
  pm_math_Quaternion_xform_ra(xx + 344, xx + 408, xx + 411);
  xx[147] = xx[144] * motionData[333];
  xx[151] = xx[144] * motionData[334];
  xx[301] = xx[160] + xx[340];
  xx[408] = - xx[147];
  xx[409] = - xx[151];
  xx[410] = xx[301];
  pm_math_Vector3_cross_ra(xx + 353, xx + 408, xx + 414);
  xx[160] = xx[389] * xx[144];
  xx[302] = xx[363] * xx[144];
  xx[340] = xx[391] + xx[367];
  xx[408] = xx[160];
  xx[409] = xx[302];
  xx[410] = - xx[340];
  pm_math_Vector3_cross_ra(xx + 353, xx + 408, xx + 417);
  xx[349] = xx[18] * (xx[417] + xx[160] * xx[34]);
  xx[160] = xx[18] * (xx[414] - xx[147] * xx[34]) + xx[349] - motionData[334];
  xx[420] = motionData[112];
  xx[421] = motionData[113];
  xx[422] = motionData[114];
  xx[423] = motionData[115];
  xx[424] = state[112];
  xx[425] = state[113];
  xx[426] = state[114];
  xx[427] = state[115];
  xx[147] = 0.9999188709407687;
  xx[381] = 1.729133611397341e-4;
  xx[382] = 0.01273662563857442;
  xx[391] = 2.20251143642801e-6;
  xx[428] = xx[147];
  xx[429] = - xx[381];
  xx[430] = xx[382];
  xx[431] = - xx[391];
  pm_math_Quaternion_composeInverse_ra(xx + 424, xx + 428, xx + 432);
  pm_math_Quaternion_compose_ra(xx + 420, xx + 432, xx + 424);
  xx[428] = motionData[133];
  xx[429] = motionData[134];
  xx[430] = motionData[135];
  xx[431] = motionData[136];
  pm_math_Quaternion_compose_ra(xx + 424, xx + 428, xx + 436);
  xx[400] = 0.9996755567249835;
  xx[401] = 0.02547118389454986;
  xx[408] = xx[400];
  xx[409] = - 8.809330994983285e-6;
  xx[410] = - xx[401];
  pm_math_Quaternion_inverseXform_ra(xx + 428, xx + 408, xx + 424);
  pm_math_Vector3_cross_ra(xx + 424, xx + 337, xx + 440);
  pm_math_Quaternion_xform_ra(xx + 436, xx + 440, xx + 443);
  xx[440] = motionData[137];
  xx[441] = motionData[138];
  xx[442] = motionData[139];
  pm_math_Vector3_cross_ra(xx + 408, xx + 440, xx + 446);
  pm_math_Quaternion_xform_ra(xx + 432, xx + 446, xx + 408);
  xx[446] = - 1.565104140975318e-3;
  xx[447] = 0.01752317107628744;
  xx[448] = - 0.06143219441052305;
  pm_math_Quaternion_xform_ra(xx + 432, xx + 446, xx + 449);
  xx[446] = xx[408] + xx[449];
  xx[447] = xx[409] + xx[450];
  xx[448] = xx[410] + xx[451];
  pm_math_Quaternion_xform_ra(xx + 420, xx + 446, xx + 408);
  xx[446] = motionData[134];
  xx[447] = motionData[135];
  xx[448] = motionData[136];
  xx[427] = 3.458547708100935e-4;
  xx[449] = 0.9999999401922369;
  xx[450] = xx[427] * motionData[135] + xx[449] * motionData[136];
  xx[451] = xx[427] * motionData[134];
  xx[452] = - xx[450];
  xx[453] = xx[451];
  xx[454] = xx[449] * motionData[134];
  pm_math_Vector3_cross_ra(xx + 446, xx + 452, xx + 455);
  xx[446] = xx[18] * (xx[455] + xx[450] * motionData[133]);
  xx[447] = xx[449] + (xx[456] - xx[451] * motionData[133]) * xx[18];
  xx[448] = xx[18] * (xx[457] - xx[449] * motionData[133] * motionData[134]) -
    xx[427];
  pm_math_Vector3_cross_ra(xx + 446, xx + 337, xx + 450);
  pm_math_Quaternion_xform_ra(xx + 436, xx + 450, xx + 453);
  xx[450] = xx[449] * motionData[139] + xx[427] * motionData[138];
  xx[451] = - (xx[427] * motionData[137]);
  xx[452] = - (xx[449] * motionData[137]);
  pm_math_Quaternion_xform_ra(xx + 432, xx + 450, xx + 456);
  xx[450] = - 0.0177372062878282;
  xx[451] = - 2.539611720865432e-6;
  xx[452] = - 7.342999962176374e-3;
  pm_math_Quaternion_xform_ra(xx + 432, xx + 450, xx + 459);
  xx[450] = xx[456] + xx[459];
  xx[451] = xx[457] + xx[460];
  xx[452] = xx[458] + xx[461];
  pm_math_Quaternion_xform_ra(xx + 420, xx + 450, xx + 456);
  xx[427] = 0.02547118541792448;
  xx[450] = 0.9996754969366247;
  xx[459] = xx[427];
  xx[460] = 3.457425605555716e-4;
  xx[461] = xx[450];
  pm_math_Quaternion_inverseXform_ra(xx + 428, xx + 459, xx + 462);
  pm_math_Vector3_cross_ra(xx + 462, xx + 337, xx + 428);
  pm_math_Quaternion_xform_ra(xx + 436, xx + 428, xx + 337);
  pm_math_Vector3_cross_ra(xx + 459, xx + 440, xx + 428);
  pm_math_Quaternion_xform_ra(xx + 432, xx + 428, xx + 440);
  xx[428] = 0.06142613026408447;
  xx[429] = 7.791863919619067e-3;
  xx[430] = - 1.567799088051977e-3;
  pm_math_Quaternion_xform_ra(xx + 432, xx + 428, xx + 459);
  xx[428] = xx[440] + xx[459];
  xx[429] = xx[441] + xx[460];
  xx[430] = xx[442] + xx[461];
  pm_math_Quaternion_xform_ra(xx + 420, xx + 428, xx + 431);
  xx[465] = motionData[245];
  xx[466] = motionData[246];
  xx[467] = motionData[247];
  xx[468] = motionData[248];
  xx[428] = - xx[147];
  xx[429] = - xx[382];
  xx[469] = xx[428];
  xx[470] = xx[381];
  xx[471] = xx[429];
  xx[472] = xx[391];
  xx[473] = - state[119];
  xx[474] = - state[120];
  xx[475] = - state[121];
  xx[476] = - state[122];
  pm_math_Quaternion_compose_ra(xx + 469, xx + 473, xx + 477);
  pm_math_Quaternion_compose_ra(xx + 465, xx + 477, xx + 469);
  xx[381] = xx[328] * xx[471];
  xx[391] = xx[336] * xx[472];
  xx[430] = xx[381] - xx[391];
  xx[434] = xx[328] * xx[470];
  xx[440] = xx[430];
  xx[441] = - xx[434];
  xx[442] = xx[336] * xx[470];
  pm_math_Vector3_cross_ra(xx + 470, xx + 440, xx + 459);
  xx[435] = xx[303] * xx[470];
  xx[440] = xx[435] - xx[391];
  xx[391] = xx[336] * xx[471];
  xx[465] = - (xx[303] * xx[471]);
  xx[466] = xx[440];
  xx[467] = xx[391];
  pm_math_Vector3_cross_ra(xx + 470, xx + 465, xx + 473);
  xx[441] = xx[469] * xx[471];
  xx[442] = xx[303] * xx[472];
  xx[451] = xx[435] + xx[381];
  xx[465] = - xx[442];
  xx[466] = - (xx[328] * xx[472]);
  xx[467] = xx[451];
  pm_math_Vector3_cross_ra(xx + 470, xx + 465, xx + 476);
  xx[381] = xx[378] * xx[34];
  xx[435] = xx[18] * (xx[384] + xx[381]);
  xx[452] = xx[18] * (xx[365] - xx[359] * xx[34]) + xx[435] - motionData[335];
  xx[359] = (xx[398] * xx[34] + xx[403]) * xx[18];
  xx[398] = (xx[192] * xx[34] + xx[396]) * xx[18] + xx[359];
  xx[192] = xx[302] * xx[34];
  xx[465] = (xx[192] + xx[418]) * xx[18];
  xx[466] = motionData[333] + xx[18] * (xx[415] - xx[151] * xx[34]) + xx[465];
  xx[151] = xx[469] * xx[472];
  xx[364] = (xx[385] - xx[379] * xx[34]) * xx[18];
  xx[365] = motionData[334] + xx[18] * (xx[366] - xx[360] * xx[34]) + xx[364];
  xx[360] = xx[18] * (xx[404] - xx[362] * xx[34]);
  xx[362] = xx[18] * (xx[397] - xx[348] * xx[34]) + xx[360] - motionData[333];
  xx[348] = (xx[419] - xx[34] * xx[340]) * xx[18];
  xx[340] = (xx[301] * xx[34] + xx[416]) * xx[18] + xx[348];
  xx[301] = xx[469] * xx[470];
  xx[366] = 0.15472;
  xx[379] = 0.29499;
  xx[383] = 2.200000000000003e-3;
  xx[395] = - xx[366];
  xx[396] = xx[379];
  xx[397] = - xx[383];
  pm_math_Vector3_cross_ra(xx + 333, xx + 395, xx + 402);
  pm_math_Quaternion_xform_ra(xx + 344, xx + 402, xx + 333);
  pm_math_Vector3_cross_ra(xx + 386, xx + 395, xx + 402);
  pm_math_Quaternion_xform_ra(xx + 344, xx + 402, xx + 384);
  pm_math_Vector3_cross_ra(xx + 405, xx + 395, xx + 402);
  pm_math_Quaternion_xform_ra(xx + 344, xx + 402, xx + 405);
  pm_math_Vector3_cross_ra(xx + 424, xx + 395, xx + 344);
  pm_math_Quaternion_xform_ra(xx + 436, xx + 344, xx + 402);
  pm_math_Vector3_cross_ra(xx + 446, xx + 395, xx + 344);
  pm_math_Quaternion_xform_ra(xx + 436, xx + 344, xx + 414);
  pm_math_Vector3_cross_ra(xx + 462, xx + 395, xx + 344);
  pm_math_Quaternion_xform_ra(xx + 436, xx + 344, xx + 395);
  xx[344] = xx[379] * xx[471];
  xx[345] = xx[383] * xx[472];
  xx[346] = xx[344] - xx[345];
  xx[347] = xx[379] * xx[470];
  xx[417] = xx[346];
  xx[418] = - xx[347];
  xx[419] = xx[383] * xx[470];
  pm_math_Vector3_cross_ra(xx + 470, xx + 417, xx + 424);
  xx[387] = xx[366] * xx[470];
  xx[388] = xx[345] + xx[387];
  xx[345] = xx[383] * xx[471];
  xx[417] = xx[366] * xx[471];
  xx[418] = - xx[388];
  xx[419] = xx[345];
  pm_math_Vector3_cross_ra(xx + 470, xx + 417, xx + 436);
  xx[417] = xx[366] * xx[472];
  xx[418] = xx[344] - xx[387];
  xx[446] = xx[417];
  xx[447] = - (xx[379] * xx[472]);
  xx[448] = xx[418];
  pm_math_Vector3_cross_ra(xx + 470, xx + 446, xx + 462);
  xx[344] = 1.729133611399561e-4;
  xx[387] = 2.202511436425175e-6;
  xx[479] = xx[428];
  xx[480] = - xx[344];
  xx[481] = xx[429];
  xx[482] = - xx[387];
  xx[483] = - state[13];
  xx[484] = - state[14];
  xx[485] = - state[15];
  xx[486] = - state[16];
  pm_math_Quaternion_compose_ra(xx + 479, xx + 483, xx + 487);
  xx[419] = xx[328] * xx[489];
  xx[428] = xx[336] * xx[490];
  xx[429] = xx[419] + xx[428];
  xx[439] = xx[328] * xx[488];
  xx[446] = - xx[429];
  xx[447] = xx[439];
  xx[448] = xx[336] * xx[488];
  pm_math_Vector3_cross_ra(xx + 488, xx + 446, xx + 470);
  xx[446] = xx[303] * xx[488];
  xx[447] = xx[446] - xx[428];
  xx[428] = xx[336] * xx[489];
  xx[479] = - (xx[303] * xx[489]);
  xx[480] = xx[447];
  xx[481] = xx[428];
  pm_math_Vector3_cross_ra(xx + 488, xx + 479, xx + 482);
  xx[448] = xx[487] * xx[489];
  xx[467] = xx[303] * xx[490];
  xx[468] = xx[446] - xx[419];
  xx[479] = - xx[467];
  xx[480] = xx[328] * xx[490];
  xx[481] = xx[468];
  pm_math_Vector3_cross_ra(xx + 488, xx + 479, xx + 491);
  xx[494] = xx[147];
  xx[495] = xx[344];
  xx[496] = xx[382];
  xx[497] = xx[387];
  xx[498] = state[20];
  xx[499] = - state[21];
  xx[500] = - state[22];
  xx[501] = - state[23];
  pm_math_Quaternion_compose_ra(xx + 494, xx + 498, xx + 502);
  xx[494] = motionData[21];
  xx[495] = motionData[22];
  xx[496] = motionData[23];
  xx[497] = motionData[24];
  pm_math_Quaternion_compose_ra(xx + 502, xx + 494, xx + 498);
  xx[479] = - xx[400];
  xx[480] = - 8.80933099498327e-6;
  xx[481] = xx[401];
  pm_math_Quaternion_inverseXform_ra(xx + 502, xx + 479, xx + 506);
  pm_math_Quaternion_inverseXform_ra(xx + 494, xx + 506, xx + 479);
  xx[509] = 0.8225246892800159;
  xx[510] = 0.264;
  xx[511] = - 0.07704855223048318;
  pm_math_Vector3_cross_ra(xx + 479, xx + 509, xx + 512);
  pm_math_Quaternion_xform_ra(xx + 498, xx + 512, xx + 515);
  xx[512] = motionData[25];
  xx[513] = motionData[26];
  xx[514] = motionData[27];
  pm_math_Vector3_cross_ra(xx + 506, xx + 512, xx + 518);
  pm_math_Quaternion_xform_ra(xx + 502, xx + 518, xx + 506);
  xx[147] = 3.458547708105375e-4;
  xx[344] = xx[449] * xx[505] - xx[147] * xx[504];
  xx[382] = xx[147] * xx[503];
  xx[518] = xx[344];
  xx[519] = xx[382];
  xx[520] = - (xx[449] * xx[503]);
  pm_math_Vector3_cross_ra(xx + 503, xx + 518, xx + 521);
  xx[518] = xx[18] * (xx[521] - xx[502] * xx[344]);
  xx[519] = (xx[522] - xx[502] * xx[382]) * xx[18] - xx[449];
  xx[520] = xx[18] * (xx[523] + xx[449] * xx[502] * xx[503]) - xx[147];
  pm_math_Quaternion_inverseXform_ra(xx + 494, xx + 518, xx + 521);
  pm_math_Vector3_cross_ra(xx + 521, xx + 509, xx + 524);
  pm_math_Quaternion_xform_ra(xx + 498, xx + 524, xx + 527);
  pm_math_Vector3_cross_ra(xx + 518, xx + 512, xx + 524);
  pm_math_Quaternion_xform_ra(xx + 502, xx + 524, xx + 518);
  xx[524] = - xx[427];
  xx[525] = 3.457425605560157e-4;
  xx[526] = - xx[450];
  pm_math_Quaternion_inverseXform_ra(xx + 502, xx + 524, xx + 530);
  pm_math_Quaternion_inverseXform_ra(xx + 494, xx + 530, xx + 524);
  pm_math_Vector3_cross_ra(xx + 524, xx + 509, xx + 533);
  pm_math_Quaternion_xform_ra(xx + 498, xx + 533, xx + 536);
  pm_math_Vector3_cross_ra(xx + 530, xx + 512, xx + 533);
  pm_math_Quaternion_xform_ra(xx + 502, xx + 533, xx + 512);
  xx[502] = motionData[14];
  xx[503] = motionData[15];
  xx[504] = motionData[16];
  xx[505] = motionData[17];
  xx[147] = xx[14] * state[27] + xx[15] * state[29];
  xx[344] = xx[14] * state[28] - xx[15] * state[30];
  xx[382] = xx[14] * state[29] - xx[15] * state[27];
  xx[387] = xx[15] * state[28] + xx[14] * state[30];
  xx[530] = - xx[147];
  xx[531] = xx[344];
  xx[532] = xx[382];
  xx[533] = xx[387];
  pm_math_Quaternion_compose_ra(xx + 502, xx + 530, xx + 539);
  xx[14] = xx[382] * xx[382];
  xx[15] = xx[387] * xx[387];
  xx[400] = xx[147] * xx[387];
  xx[401] = xx[344] * xx[382];
  xx[419] = xx[382] * xx[147];
  xx[427] = xx[344] * xx[387];
  xx[543] = (xx[14] + xx[15]) * xx[18] - xx[153];
  xx[544] = - ((xx[400] + xx[401]) * xx[18]);
  xx[545] = xx[18] * (xx[419] - xx[427]);
  pm_math_Vector3_cross_ra(xx + 543, xx + 509, xx + 546);
  pm_math_Quaternion_xform_ra(xx + 539, xx + 546, xx + 549);
  xx[446] = 0.2717;
  xx[546] = - 0.8116895113357588;
  xx[547] = - xx[446];
  xx[548] = - 0.03913410103116464;
  pm_math_Vector3_cross_ra(xx + 543, xx + 546, xx + 552);
  pm_math_Quaternion_xform_ra(xx + 530, xx + 552, xx + 555);
  pm_math_Quaternion_xform_ra(xx + 502, xx + 555, xx + 552);
  xx[449] = xx[344] * xx[344];
  xx[450] = xx[344] * xx[147];
  xx[147] = xx[382] * xx[387];
  xx[558] = xx[18] * (xx[400] - xx[401]);
  xx[559] = (xx[15] + xx[449]) * xx[18] - xx[153];
  xx[560] = - ((xx[450] + xx[147]) * xx[18]);
  pm_math_Vector3_cross_ra(xx + 558, xx + 509, xx + 561);
  pm_math_Quaternion_xform_ra(xx + 539, xx + 561, xx + 564);
  pm_math_Vector3_cross_ra(xx + 558, xx + 546, xx + 561);
  pm_math_Quaternion_xform_ra(xx + 530, xx + 561, xx + 567);
  pm_math_Quaternion_xform_ra(xx + 502, xx + 567, xx + 561);
  xx[570] = - ((xx[419] + xx[427]) * xx[18]);
  xx[571] = xx[18] * (xx[450] - xx[147]);
  xx[572] = (xx[449] + xx[14]) * xx[18] - xx[153];
  pm_math_Vector3_cross_ra(xx + 570, xx + 509, xx + 573);
  pm_math_Quaternion_xform_ra(xx + 539, xx + 573, xx + 509);
  pm_math_Vector3_cross_ra(xx + 570, xx + 546, xx + 573);
  pm_math_Quaternion_xform_ra(xx + 530, xx + 573, xx + 546);
  pm_math_Quaternion_xform_ra(xx + 502, xx + 546, xx + 573);
  xx[14] = xx[487] * xx[490];
  xx[15] = xx[487] * xx[488];
  xx[147] = xx[379] * xx[489];
  xx[153] = xx[383] * xx[490];
  xx[344] = xx[147] + xx[153];
  xx[382] = xx[379] * xx[488];
  xx[502] = - xx[344];
  xx[503] = xx[382];
  xx[504] = xx[383] * xx[488];
  pm_math_Vector3_cross_ra(xx + 488, xx + 502, xx + 576);
  xx[387] = xx[366] * xx[488];
  xx[400] = xx[153] + xx[387];
  xx[153] = xx[383] * xx[489];
  xx[502] = xx[366] * xx[489];
  xx[503] = - xx[400];
  xx[504] = xx[153];
  pm_math_Vector3_cross_ra(xx + 488, xx + 502, xx + 579);
  xx[401] = xx[366] * xx[490];
  xx[419] = xx[387] + xx[147];
  xx[502] = xx[401];
  xx[503] = xx[379] * xx[490];
  xx[504] = - xx[419];
  pm_math_Vector3_cross_ra(xx + 488, xx + 502, xx + 582);
  xx[488] = 0.5299620178672857;
  xx[489] = 0.268;
  xx[490] = - 0.08339940224027326;
  pm_math_Vector3_cross_ra(xx + 479, xx + 488, xx + 502);
  pm_math_Quaternion_xform_ra(xx + 498, xx + 502, xx + 479);
  pm_math_Vector3_cross_ra(xx + 521, xx + 488, xx + 502);
  pm_math_Quaternion_xform_ra(xx + 498, xx + 502, xx + 521);
  pm_math_Vector3_cross_ra(xx + 524, xx + 488, xx + 502);
  pm_math_Quaternion_xform_ra(xx + 498, xx + 502, xx + 524);
  pm_math_Vector3_cross_ra(xx + 543, xx + 488, xx + 498);
  pm_math_Quaternion_xform_ra(xx + 539, xx + 498, xx + 501);
  pm_math_Vector3_cross_ra(xx + 558, xx + 488, xx + 498);
  pm_math_Quaternion_xform_ra(xx + 539, xx + 498, xx + 585);
  pm_math_Vector3_cross_ra(xx + 570, xx + 488, xx + 498);
  pm_math_Quaternion_xform_ra(xx + 539, xx + 498, xx + 488);
  xx[147] = 0.04444999999999999;
  xx[387] = xx[147] * xx[144];
  xx[427] = xx[367] - xx[387];
  xx[449] = xx[35] * xx[147];
  xx[498] = xx[427];
  xx[499] = - xx[378];
  xx[500] = xx[449];
  pm_math_Vector3_cross_ra(xx + 353, xx + 498, xx + 539);
  xx[378] = 0.14521;
  xx[450] = xx[68] * xx[378];
  xx[485] = xx[35] * xx[378];
  xx[486] = xx[387] + xx[485];
  xx[387] = xx[68] * xx[147];
  xx[498] = xx[450];
  xx[499] = - xx[486];
  xx[500] = xx[387];
  pm_math_Vector3_cross_ra(xx + 353, xx + 498, xx + 588);
  xx[147] = xx[378] * xx[144];
  xx[378] = xx[367] - xx[485];
  xx[498] = xx[147];
  xx[499] = - xx[302];
  xx[500] = xx[378];
  pm_math_Vector3_cross_ra(xx + 353, xx + 498, xx + 591);
  xx[302] = 0.288;
  xx[498] = 0.5235527553923325;
  xx[499] = xx[446];
  xx[500] = 0.03764753640142538;
  pm_math_Vector3_cross_ra(xx + 543, xx + 498, xx + 594);
  pm_math_Quaternion_xform_ra(xx + 530, xx + 594, xx + 597);
  pm_math_Vector3_cross_ra(xx + 558, xx + 498, xx + 594);
  pm_math_Quaternion_xform_ra(xx + 530, xx + 594, xx + 600);
  pm_math_Vector3_cross_ra(xx + 570, xx + 498, xx + 594);
  pm_math_Quaternion_xform_ra(xx + 530, xx + 594, xx + 498);
  xx[603] = motionData[98];
  xx[604] = motionData[99];
  xx[605] = motionData[100];
  xx[606] = motionData[101];
  pm_math_Quaternion_compose_ra(xx + 530, xx + 603, xx + 607);
  xx[367] = 3.749811151839773e-3;
  pm_math_Quaternion_inverseXform_ra(xx + 603, xx + 543, xx + 594);
  xx[446] = 0.05742288587815435;
  xx[611] = xx[367] * xx[595] - xx[446] * xx[596];
  xx[612] = - (xx[367] * xx[594]);
  xx[613] = xx[446] * xx[594];
  pm_math_Quaternion_xform_ra(xx + 607, xx + 611, xx + 594);
  xx[611] = motionData[102];
  xx[612] = motionData[103];
  xx[613] = motionData[104];
  pm_math_Vector3_cross_ra(xx + 543, xx + 611, xx + 614);
  pm_math_Quaternion_xform_ra(xx + 530, xx + 614, xx + 542);
  pm_math_Quaternion_inverseXform_ra(xx + 603, xx + 558, xx + 614);
  xx[617] = xx[367] * xx[615] - xx[446] * xx[616];
  xx[618] = - (xx[367] * xx[614]);
  xx[619] = xx[446] * xx[614];
  pm_math_Quaternion_xform_ra(xx + 607, xx + 617, xx + 614);
  pm_math_Vector3_cross_ra(xx + 558, xx + 611, xx + 617);
  pm_math_Quaternion_xform_ra(xx + 530, xx + 617, xx + 558);
  pm_math_Quaternion_inverseXform_ra(xx + 603, xx + 570, xx + 617);
  xx[603] = xx[367] * xx[618] - xx[446] * xx[619];
  xx[604] = - (xx[367] * xx[617]);
  xx[605] = xx[446] * xx[617];
  pm_math_Quaternion_xform_ra(xx + 607, xx + 603, xx + 617);
  pm_math_Vector3_cross_ra(xx + 570, xx + 611, xx + 603);
  pm_math_Quaternion_xform_ra(xx + 530, xx + 603, xx + 570);
  xx[485] = 0.9996758317937826;
  xx[504] = xx[2] * state[94];
  xx[505] = cos(xx[504]);
  xx[530] = 0.01780577773926495;
  xx[531] = sin(xx[504]);
  xx[504] = xx[485] * xx[505] + xx[530] * xx[531];
  xx[532] = xx[485] * xx[531] - xx[530] * xx[505];
  xx[533] = 0.01819561954822274;
  xx[534] = 3.240922178968028e-4;
  xx[535] = xx[533] * xx[505] + xx[534] * xx[531];
  xx[545] = - xx[535];
  xx[603] = xx[533] * xx[531] - xx[534] * xx[505];
  xx[604] = xx[504];
  xx[605] = xx[532];
  xx[606] = xx[545];
  xx[607] = xx[603];
  pm_math_Quaternion_compose_ra(xx + 494, xx + 604, xx + 608);
  xx[505] = xx[446] * xx[610] + xx[367] * xx[611];
  xx[531] = xx[446] * xx[609];
  xx[604] = xx[505];
  xx[605] = - xx[531];
  xx[606] = - (xx[367] * xx[609]);
  pm_math_Vector3_cross_ra(xx + 609, xx + 604, xx + 620);
  xx[604] = 5.002590631222067e-3;
  xx[605] = 0.05733821843436025;
  xx[606] = xx[604] * xx[603] - xx[535] * xx[605];
  xx[610] = xx[532];
  xx[611] = xx[545];
  xx[612] = xx[603];
  xx[535] = xx[532] * xx[605];
  xx[545] = xx[532] * xx[604];
  xx[623] = xx[606];
  xx[624] = - xx[535];
  xx[625] = - xx[545];
  pm_math_Vector3_cross_ra(xx + 610, xx + 623, xx + 626);
  xx[610] = (xx[606] * xx[504] + xx[626]) * xx[18];
  xx[611] = xx[18] * (xx[627] - xx[535] * xx[504]) - xx[604];
  xx[612] = xx[605] + (xx[628] - xx[545] * xx[504]) * xx[18];
  pm_math_Quaternion_xform_ra(xx + 494, xx + 610, xx + 623);
  xx[494] = state[127];
  xx[495] = state[128];
  xx[496] = state[129];
  xx[497] = 0.17753;
  xx[504] = xx[497] * state[128];
  xx[532] = 0.15579;
  xx[535] = xx[532] * state[129];
  xx[545] = xx[504] + xx[535];
  xx[603] = xx[497] * state[127];
  xx[610] = - xx[545];
  xx[611] = xx[603];
  xx[612] = xx[532] * state[127];
  pm_math_Vector3_cross_ra(xx + 494, xx + 610, xx + 626);
  xx[606] = 5.789999999999964e-3;
  xx[607] = xx[606] * state[127];
  xx[610] = xx[607] - xx[535];
  xx[535] = xx[532] * state[128];
  xx[611] = - (xx[606] * state[128]);
  xx[612] = xx[610];
  xx[613] = xx[535];
  pm_math_Vector3_cross_ra(xx + 494, xx + 611, xx + 629);
  xx[611] = xx[606] * state[129];
  xx[612] = xx[607] - xx[504];
  xx[632] = - xx[611];
  xx[633] = xx[497] * state[129];
  xx[634] = xx[612];
  pm_math_Vector3_cross_ra(xx + 494, xx + 632, xx + 635);
  xx[494] = xx[2] * state[96];
  xx[2] = cos(xx[494]);
  xx[495] = sin(xx[494]);
  xx[494] = xx[485] * xx[2] - xx[530] * xx[495];
  xx[496] = xx[534] * xx[2] + xx[533] * xx[495];
  xx[504] = xx[534] * xx[495] - xx[533] * xx[2];
  xx[533] = xx[367] * xx[496] - xx[504] * xx[446];
  xx[534] = xx[485] * xx[495] + xx[530] * xx[2];
  xx[632] = xx[534];
  xx[633] = xx[504];
  xx[634] = xx[496];
  xx[2] = xx[534] * xx[446];
  xx[485] = xx[534] * xx[367];
  xx[638] = xx[533];
  xx[639] = xx[2];
  xx[640] = - xx[485];
  pm_math_Vector3_cross_ra(xx + 632, xx + 638, xx + 641);
  xx[495] = xx[604] * xx[496] - xx[504] * xx[605];
  xx[496] = xx[534] * xx[605];
  xx[504] = xx[534] * xx[604];
  xx[638] = xx[495];
  xx[639] = xx[496];
  xx[640] = - xx[504];
  pm_math_Vector3_cross_ra(xx + 632, xx + 638, xx + 644);
  xx[530] = xx[68] * motionData[124];
  xx[534] = xx[144] * motionData[125];
  xx[604] = xx[530] + xx[534];
  xx[605] = xx[35] * motionData[124];
  xx[607] = xx[35] * motionData[125];
  xx[632] = xx[604];
  xx[633] = - xx[605];
  xx[634] = - xx[607];
  pm_math_Vector3_cross_ra(xx + 353, xx + 632, xx + 638);
  xx[613] = xx[68] * motionData[123];
  xx[632] = xx[35] * motionData[123];
  xx[35] = xx[534] + xx[632];
  xx[534] = xx[68] * motionData[125];
  xx[647] = - xx[613];
  xx[648] = xx[35];
  xx[649] = - xx[534];
  pm_math_Vector3_cross_ra(xx + 353, xx + 647, xx + 650);
  xx[68] = xx[144] * motionData[123];
  xx[633] = xx[144] * motionData[124];
  xx[144] = xx[632] + xx[530];
  xx[647] = - xx[68];
  xx[648] = - xx[633];
  xx[649] = xx[144];
  pm_math_Vector3_cross_ra(xx + 353, xx + 647, xx + 653);
  xx[353] = xx[497] * state[107];
  xx[354] = xx[532] * state[108];
  xx[355] = xx[353] - xx[354];
  xx[647] = state[106];
  xx[648] = state[107];
  xx[649] = state[108];
  xx[530] = xx[497] * state[106];
  xx[656] = xx[355];
  xx[657] = - xx[530];
  xx[658] = xx[532] * state[106];
  pm_math_Vector3_cross_ra(xx + 647, xx + 656, xx + 659);
  xx[656] = (xx[355] * state[105] + xx[659]) * xx[18];
  xx[657] = xx[532] + xx[18] * (xx[660] - xx[530] * state[105]);
  xx[658] = xx[497] + (xx[532] * state[105] * state[106] + xx[661]) * xx[18];
  pm_math_Quaternion_xform_ra(xx + 420, xx + 656, xx + 659);
  xx[355] = xx[606] * state[106];
  xx[530] = xx[355] - xx[354];
  xx[354] = xx[532] * state[107];
  xx[656] = - (xx[606] * state[107]);
  xx[657] = xx[530];
  xx[658] = xx[354];
  pm_math_Vector3_cross_ra(xx + 647, xx + 656, xx + 662);
  xx[656] = (xx[662] - xx[606] * state[105] * state[107]) * xx[18] - xx[532];
  xx[657] = (xx[530] * state[105] + xx[663]) * xx[18];
  xx[658] = xx[18] * (xx[664] + xx[354] * state[105]) - xx[606];
  pm_math_Quaternion_xform_ra(xx + 420, xx + 656, xx + 662);
  xx[354] = xx[606] * state[108];
  xx[530] = xx[355] + xx[353];
  xx[656] = - xx[354];
  xx[657] = - (xx[497] * state[108]);
  xx[658] = xx[530];
  pm_math_Vector3_cross_ra(xx + 647, xx + 656, xx + 665);
  xx[647] = xx[18] * (xx[665] - xx[354] * state[105]) - xx[497];
  xx[648] = xx[606] + (xx[666] - xx[497] * state[105] * state[108]) * xx[18];
  xx[649] = (xx[530] * state[105] + xx[667]) * xx[18];
  pm_math_Quaternion_xform_ra(xx + 420, xx + 647, xx + 353);
  J[15] = - ((xx[3] * xx[13] + xx[21]) * xx[18] + (xx[24] * xx[3] + xx[28]) *
             xx[18]);
  J[29] = (xx[27] * xx[31] + xx[41]) * xx[18] + xx[48];
  J[30] = motionData[83] + xx[18] * (xx[57] - xx[45] * xx[31]) + xx[62] + xx[38];
  J[31] = xx[18] * (xx[71] - xx[55] * xx[31]) + xx[44] - motionData[82] + xx[40];
  J[32] = xx[74];
  J[33] = xx[93];
  J[34] = xx[88];
  J[73] = - (xx[18] * (xx[22] + xx[4] * xx[3]) + xx[18] * (xx[29] - xx[10] * xx
              [3]) - xx[56]);
  J[87] = xx[18] * (xx[42] - xx[36] * xx[31]) + xx[70] - motionData[83] - xx[38];
  J[88] = (xx[50] * xx[31] + xx[58]) * xx[18] + xx[46];
  J[89] = motionData[81] + xx[18] * (xx[72] - xx[63] * xx[31]) + xx[60] - xx[54];
  J[90] = xx[75];
  J[91] = xx[94];
  J[92] = xx[89];
  J[131] = - ((xx[23] - xx[6] * xx[3]) * xx[18] + (xx[30] - xx[11] * xx[3]) *
              xx[18] + xx[49]);
  J[145] = motionData[82] + xx[18] * (xx[43] - xx[37] * xx[31]) + xx[52] - xx[40];
  J[146] = xx[18] * (xx[59] - xx[26] * xx[31]) + xx[65] - motionData[81] + xx[54];
  J[147] = (xx[64] * xx[31] + xx[73]) * xx[18] + xx[66];
  J[148] = xx[76];
  J[149] = xx[95];
  J[150] = xx[90];
  J[190] = - ((xx[8] * xx[9] + xx[99]) * xx[18] + (xx[9] * xx[7] + xx[96]) * xx
              [18]);
  J[191] = (xx[78] * xx[79] + xx[108]) * xx[18] + xx[113];
  J[192] = motionData[300] + xx[18] * (xx[122] - xx[107] * xx[79]) + xx[126] -
    xx[105];
  J[193] = xx[18] * (xx[135] - xx[120] * xx[79]) + xx[92] - motionData[299] -
    xx[91];
  J[194] = xx[170];
  J[195] = xx[183];
  J[196] = xx[164];
  J[197] = xx[196];
  J[198] = xx[204];
  J[199] = xx[212];
  J[200] = xx[222];
  J[201] = xx[225];
  J[202] = xx[219];
  J[248] = - (xx[18] * (xx[100] - xx[1] * xx[9]) + xx[18] * (xx[97] + xx[12] *
    xx[9]) - xx[56]);
  J[249] = xx[18] * (xx[109] - xx[85] * xx[79]) + xx[17] - motionData[300] + xx
    [105];
  J[250] = (xx[115] * xx[79] + xx[123]) * xx[18] + xx[69];
  J[251] = motionData[298] + xx[18] * (xx[136] - xx[127] * xx[79]) + xx[87] -
    xx[119];
  J[252] = xx[171];
  J[253] = xx[184];
  J[254] = xx[165];
  J[255] = xx[197];
  J[256] = xx[205];
  J[257] = xx[213];
  J[258] = xx[223];
  J[259] = xx[226];
  J[260] = xx[220];
  J[306] = - ((xx[101] - xx[5] * xx[9]) * xx[18] + (xx[98] - xx[19] * xx[9]) *
              xx[18] - xx[49]);
  J[307] = motionData[299] + xx[18] * (xx[110] - xx[86] * xx[79]) + xx[111] +
    xx[91];
  J[308] = xx[18] * (xx[124] - xx[77] * xx[79]) + xx[112] - motionData[298] +
    xx[119];
  J[309] = (xx[128] * xx[79] + xx[137]) * xx[18] + xx[106];
  J[310] = xx[172];
  J[311] = xx[185];
  J[312] = xx[166];
  J[313] = xx[198];
  J[314] = xx[206];
  J[315] = xx[214];
  J[316] = xx[224];
  J[317] = xx[227];
  J[318] = xx[221];
  J[377] = xx[193] + xx[169];
  J[378] = xx[235] + xx[203] + xx[38];
  J[379] = xx[247] + xx[117] + xx[40];
  J[383] = xx[260] + xx[269];
  J[384] = xx[278] + xx[275];
  J[385] = xx[132] + xx[281];
  J[386] = (xx[284] * xx[81] + xx[288]) * xx[18];
  J[387] = xx[105] + (xx[291] - xx[119] * xx[125]) * xx[18];
  J[388] = xx[18] * (xx[294] - xx[284] * xx[141]) - xx[91];
  J[435] = xx[194] + xx[67] - xx[38];
  J[436] = xx[236] + xx[83];
  J[437] = xx[248] + xx[118] - xx[54];
  J[441] = xx[261] + xx[270];
  J[442] = xx[279] + xx[276];
  J[443] = xx[133] + xx[282];
  J[444] = xx[18] * (xx[289] - xx[284] * xx[82]) - xx[105];
  J[445] = (xx[284] * xx[114] + xx[292]) * xx[18];
  J[446] = xx[119] + (xx[295] - xx[91] * xx[116]) * xx[18];
  J[493] = xx[195] + xx[152] - xx[40];
  J[494] = xx[237] + xx[167] + xx[54];
  J[495] = xx[249] + xx[142];
  J[499] = xx[262] + xx[271];
  J[500] = xx[280] + xx[277];
  J[501] = xx[134] + xx[283];
  J[502] = xx[91] + (xx[290] - xx[105] * xx[121]) * xx[18];
  J[503] = xx[18] * (xx[293] - xx[284] * xx[80]) - xx[119];
  J[504] = (xx[284] * xx[143] + xx[296]) * xx[18];
  J[551] = xx[129] + xx[169];
  J[552] = xx[215] + xx[203] + xx[38];
  J[553] = xx[238] + xx[117] + xx[40];
  J[557] = xx[241] + xx[269];
  J[558] = xx[244] + xx[275];
  J[559] = xx[231] + xx[281];
  J[560] = (xx[284] * xx[218] + xx[253]) * xx[18];
  J[561] = xx[202] + (xx[168] * xx[125] + xx[256]) * xx[18];
  J[562] = xx[18] * (xx[266] + xx[284] * xx[234]) - xx[173];
  J[609] = xx[130] + xx[67] - xx[38];
  J[610] = xx[216] + xx[83];
  J[611] = xx[239] + xx[118] - xx[54];
  J[615] = xx[242] + xx[270];
  J[616] = xx[245] + xx[276];
  J[617] = xx[232] + xx[282];
  J[618] = xx[18] * (xx[254] - xx[284] * xx[228]) - xx[202];
  J[619] = (xx[284] * xx[230] + xx[257]) * xx[18];
  J[620] = (xx[267] - xx[173] * xx[116]) * xx[18] - xx[168];
  J[667] = xx[131] + xx[152] - xx[40];
  J[668] = xx[217] + xx[167] + xx[54];
  J[669] = xx[240] + xx[142];
  J[673] = xx[243] + xx[271];
  J[674] = xx[246] + xx[277];
  J[675] = xx[233] + xx[283];
  J[676] = xx[173] + (xx[255] - xx[202] * xx[121]) * xx[18];
  J[677] = xx[168] + xx[18] * (xx[258] - xx[284] * xx[208]);
  J[678] = (xx[284] * xx[250] + xx[268]) * xx[18];
  J[713] = (xx[79] * xx[251] + xx[272]) * xx[18] + xx[113];
  J[714] = (xx[263] * xx[79] + xx[297]) * xx[18] + xx[126] + xx[285];
  J[715] = xx[18] * (xx[304] + xx[286] * xx[79]) + xx[92] + xx[229];
  J[771] = xx[18] * (xx[273] + xx[252] * xx[79]) + xx[17] - xx[285];
  J[772] = (xx[265] * xx[79] + xx[298]) * xx[18] + xx[69];
  J[773] = (xx[287] * xx[79] + xx[305]) * xx[18] + xx[87] - xx[264];
  J[829] = (xx[274] - xx[259] * xx[79]) * xx[18] + xx[111] - xx[229];
  J[830] = xx[18] * (xx[299] - xx[207] * xx[79]) + xx[112] + xx[264];
  J[831] = (xx[306] - xx[79] * xx[300]) * xx[18] + xx[106];
  J[887] = xx[325] + xx[329];
  J[888] = xx[341] + xx[20] - xx[105];
  J[889] = xx[350] + xx[102] - xx[91];
  J[890] = xx[368] + xx[161];
  J[891] = xx[374] + xx[174];
  J[892] = xx[371] + xx[138];
  J[893] = xx[154];
  J[894] = xx[157];
  J[895] = xx[189];
  J[945] = xx[326] + xx[103] + xx[105];
  J[946] = xx[342] + xx[104];
  J[947] = xx[351] + xx[84] - xx[119];
  J[948] = xx[369] + xx[162];
  J[949] = xx[375] + xx[175];
  J[950] = xx[372] + xx[139];
  J[951] = xx[155];
  J[952] = xx[158];
  J[953] = xx[190];
  J[1003] = xx[327] + xx[148] + xx[91];
  J[1004] = xx[343] + xx[149] + xx[119];
  J[1005] = xx[352] + xx[150];
  J[1006] = xx[370] + xx[163];
  J[1007] = xx[376] + xx[176];
  J[1008] = xx[373] + xx[140];
  J[1009] = xx[156];
  J[1010] = xx[159];
  J[1011] = xx[191];
  J[1061] = xx[310] + xx[329];
  J[1062] = xx[319] + xx[20] - xx[105];
  J[1063] = xx[322] + xx[102] - xx[91];
  J[1064] = xx[313] + xx[161];
  J[1065] = xx[177] + xx[174];
  J[1066] = xx[186] + xx[138];
  J[1067] = xx[180];
  J[1068] = xx[209];
  J[1069] = xx[199];
  J[1119] = xx[311] + xx[103] + xx[105];
  J[1120] = xx[320] + xx[104];
  J[1121] = xx[323] + xx[84] - xx[119];
  J[1122] = xx[314] + xx[162];
  J[1123] = xx[178] + xx[175];
  J[1124] = xx[187] + xx[139];
  J[1125] = xx[181];
  J[1126] = xx[210];
  J[1127] = xx[200];
  J[1177] = xx[312] + xx[148] + xx[91];
  J[1178] = xx[321] + xx[149] + xx[119];
  J[1179] = xx[324] + xx[150];
  J[1180] = xx[315] + xx[163];
  J[1181] = xx[179] + xx[176];
  J[1182] = xx[188] + xx[140];
  J[1183] = xx[182];
  J[1184] = xx[211];
  J[1185] = xx[201];
  J[1247] = (xx[145] * xx[31] + xx[307]) * xx[18] + xx[48];
  J[1248] = (xx[47] * xx[31] + xx[316]) * xx[18] + xx[62];
  J[1249] = xx[18] * (xx[330] + xx[39] * xx[31]) + xx[44] + xx[16];
  J[1305] = xx[18] * (xx[308] - xx[146] * xx[31]) + xx[70];
  J[1306] = (xx[317] - xx[31] * xx[32]) * xx[18] + xx[46];
  J[1307] = (xx[331] - xx[61] * xx[31]) * xx[18] + xx[60] - xx[33];
  J[1363] = (xx[51] + xx[309]) * xx[18] + xx[52] - xx[16];
  J[1364] = xx[18] * (xx[318] + xx[53]) + xx[65] + xx[33];
  J[1365] = (xx[31] * xx[25] + xx[332]) * xx[18] + xx[66];
  J[1435] = xx[356] + xx[377];
  J[1436] = xx[392] + xx[390] + xx[361];
  J[1437] = xx[411] + xx[160] + xx[363];
  J[1441] = xx[443] + xx[408];
  J[1442] = xx[453] + xx[456];
  J[1443] = xx[337] + xx[431];
  J[1444] = (xx[469] * xx[430] + xx[459]) * xx[18];
  J[1445] = (xx[473] - xx[303] * xx[441]) * xx[18] - xx[336];
  J[1446] = xx[18] * (xx[476] - xx[469] * xx[442]) - xx[328];
  J[1493] = xx[357] + xx[452] - xx[361];
  J[1494] = xx[393] + xx[398];
  J[1495] = xx[412] + xx[466] - xx[389];
  J[1499] = xx[444] + xx[409];
  J[1500] = xx[454] + xx[457];
  J[1501] = xx[338] + xx[432];
  J[1502] = xx[336] + xx[18] * (xx[460] - xx[469] * xx[434]);
  J[1503] = (xx[469] * xx[440] + xx[474]) * xx[18];
  J[1504] = xx[303] + (xx[477] - xx[328] * xx[151]) * xx[18];
  J[1551] = xx[358] + xx[365] - xx[363];
  J[1552] = xx[394] + xx[362] + xx[389];
  J[1553] = xx[413] + xx[340];
  J[1557] = xx[445] + xx[410];
  J[1558] = xx[455] + xx[458];
  J[1559] = xx[339] + xx[433];
  J[1560] = xx[328] + (xx[336] * xx[301] + xx[461]) * xx[18];
  J[1561] = xx[18] * (xx[475] + xx[469] * xx[391]) - xx[303];
  J[1562] = (xx[469] * xx[451] + xx[478]) * xx[18];
  J[1609] = xx[333] + xx[377];
  J[1610] = xx[384] + xx[390] + xx[361];
  J[1611] = xx[405] + xx[160] + xx[363];
  J[1615] = xx[402] + xx[408];
  J[1616] = xx[414] + xx[456];
  J[1617] = xx[395] + xx[431];
  J[1618] = (xx[469] * xx[346] + xx[424]) * xx[18];
  J[1619] = (xx[366] * xx[441] + xx[436]) * xx[18] - xx[383];
  J[1620] = xx[18] * (xx[462] + xx[469] * xx[417]) - xx[379];
  J[1667] = xx[334] + xx[452] - xx[361];
  J[1668] = xx[385] + xx[398];
  J[1669] = xx[406] + xx[466] - xx[389];
  J[1673] = xx[403] + xx[409];
  J[1674] = xx[415] + xx[457];
  J[1675] = xx[396] + xx[432];
  J[1676] = xx[383] + xx[18] * (xx[425] - xx[469] * xx[347]);
  J[1677] = (xx[437] - xx[469] * xx[388]) * xx[18];
  J[1678] = (xx[463] - xx[379] * xx[151]) * xx[18] - xx[366];
  J[1725] = xx[335] + xx[365] - xx[363];
  J[1726] = xx[386] + xx[362] + xx[389];
  J[1727] = xx[407] + xx[340];
  J[1731] = xx[404] + xx[410];
  J[1732] = xx[416] + xx[458];
  J[1733] = xx[397] + xx[433];
  J[1734] = xx[379] + (xx[383] * xx[301] + xx[426]) * xx[18];
  J[1735] = xx[366] + xx[18] * (xx[438] + xx[469] * xx[345]);
  J[1736] = (xx[469] * xx[418] + xx[464]) * xx[18];
  J[1746] = (xx[470] - xx[487] * xx[429]) * xx[18];
  J[1747] = (xx[482] - xx[303] * xx[448]) * xx[18] - xx[336];
  J[1748] = xx[328] + xx[18] * (xx[491] - xx[487] * xx[467]);
  J[1749] = - (xx[515] + xx[506]);
  J[1750] = - (xx[527] + xx[518]);
  J[1751] = - (xx[536] + xx[512]);
  J[1752] = - (xx[549] + xx[552]);
  J[1753] = - (xx[564] + xx[561]);
  J[1754] = - (xx[509] + xx[573]);
  J[1804] = xx[336] + xx[18] * (xx[471] + xx[487] * xx[439]);
  J[1805] = (xx[487] * xx[447] + xx[483]) * xx[18];
  J[1806] = xx[303] + (xx[328] * xx[14] + xx[492]) * xx[18];
  J[1807] = - (xx[516] + xx[507]);
  J[1808] = - (xx[528] + xx[519]);
  J[1809] = - (xx[537] + xx[513]);
  J[1810] = - (xx[550] + xx[553]);
  J[1811] = - (xx[565] + xx[562]);
  J[1812] = - (xx[510] + xx[574]);
  J[1862] = (xx[336] * xx[15] + xx[472]) * xx[18] - xx[328];
  J[1863] = xx[18] * (xx[484] + xx[487] * xx[428]) - xx[303];
  J[1864] = (xx[487] * xx[468] + xx[493]) * xx[18];
  J[1865] = - (xx[517] + xx[508]);
  J[1866] = - (xx[529] + xx[520]);
  J[1867] = - (xx[538] + xx[514]);
  J[1868] = - (xx[551] + xx[554]);
  J[1869] = - (xx[566] + xx[563]);
  J[1870] = - (xx[511] + xx[575]);
  J[1920] = (xx[576] - xx[487] * xx[344]) * xx[18];
  J[1921] = (xx[366] * xx[448] + xx[579]) * xx[18] - xx[383];
  J[1922] = xx[379] + xx[18] * (xx[582] + xx[487] * xx[401]);
  J[1923] = - (xx[479] + xx[506]);
  J[1924] = - (xx[521] + xx[518]);
  J[1925] = - (xx[524] + xx[512]);
  J[1926] = - (xx[501] + xx[552]);
  J[1927] = - (xx[585] + xx[561]);
  J[1928] = - (xx[488] + xx[573]);
  J[1978] = xx[383] + xx[18] * (xx[577] + xx[487] * xx[382]);
  J[1979] = (xx[580] - xx[487] * xx[400]) * xx[18];
  J[1980] = (xx[379] * xx[14] + xx[583]) * xx[18] - xx[366];
  J[1981] = - (xx[480] + xx[507]);
  J[1982] = - (xx[522] + xx[519]);
  J[1983] = - (xx[525] + xx[513]);
  J[1984] = - (xx[502] + xx[553]);
  J[1985] = - (xx[586] + xx[562]);
  J[1986] = - (xx[489] + xx[574]);
  J[2036] = (xx[383] * xx[15] + xx[578]) * xx[18] - xx[379];
  J[2037] = xx[366] + xx[18] * (xx[581] + xx[487] * xx[153]);
  J[2038] = (xx[584] - xx[487] * xx[419]) * xx[18];
  J[2039] = - (xx[481] + xx[508]);
  J[2040] = - (xx[523] + xx[520]);
  J[2041] = - (xx[526] + xx[514]);
  J[2042] = - (xx[503] + xx[554]);
  J[2043] = - (xx[587] + xx[563]);
  J[2044] = - (xx[490] + xx[575]);
  J[2131] = (xx[427] * xx[34] + xx[539]) * xx[18] + xx[380];
  J[2132] = (xx[450] * xx[34] + xx[588]) * xx[18] + xx[399] + xx[202];
  J[2133] = xx[18] * (xx[591] + xx[147] * xx[34]) + xx[349];
  J[2189] = xx[18] * (xx[540] - xx[381]) + xx[435] - xx[202];
  J[2190] = (xx[589] - xx[34] * xx[486]) * xx[18] + xx[359];
  J[2191] = (xx[592] - xx[192]) * xx[18] + xx[465] - xx[302];
  J[2247] = (xx[449] * xx[34] + xx[541]) * xx[18] + xx[364];
  J[2248] = xx[18] * (xx[590] + xx[387] * xx[34]) + xx[360] + xx[302];
  J[2249] = (xx[34] * xx[378] + xx[593]) * xx[18] + xx[348];
  J[2274] = - (xx[597] + xx[555]);
  J[2275] = - (xx[600] + xx[567]);
  J[2276] = - (xx[498] + xx[546]);
  J[2332] = - (xx[598] + xx[556]);
  J[2333] = - (xx[601] + xx[568]);
  J[2334] = - (xx[499] + xx[547]);
  J[2390] = - (xx[599] + xx[557]);
  J[2391] = - (xx[602] + xx[569]);
  J[2392] = - (xx[500] + xx[548]);
  J[2448] = - (xx[594] + xx[542] + xx[555]);
  J[2449] = - (xx[614] + xx[558] + xx[567]);
  J[2450] = - (xx[617] + xx[570] + xx[546]);
  J[2477] = - ((xx[608] * xx[505] + xx[620]) * xx[18] + xx[623]);
  J[2491] = (xx[626] - xx[545] * state[126]) * xx[18];
  J[2492] = (xx[629] - xx[606] * state[126] * state[128]) * xx[18] - xx[532];
  J[2493] = xx[497] + xx[18] * (xx[635] - xx[611] * state[126]);
  J[2506] = - (xx[595] + xx[543] + xx[556]);
  J[2507] = - (xx[615] + xx[559] + xx[568]);
  J[2508] = - (xx[618] + xx[571] + xx[547]);
  J[2535] = - (xx[18] * (xx[621] - xx[608] * xx[531]) + xx[624] - xx[367]);
  J[2549] = xx[532] + xx[18] * (xx[627] + xx[603] * state[126]);
  J[2550] = (xx[610] * state[126] + xx[630]) * xx[18];
  J[2551] = xx[606] + (xx[497] * state[126] * state[129] + xx[636]) * xx[18];
  J[2564] = - (xx[596] + xx[544] + xx[557]);
  J[2565] = - (xx[616] + xx[560] + xx[569]);
  J[2566] = - (xx[619] + xx[572] + xx[548]);
  J[2593] = - ((xx[622] - xx[367] * xx[608] * xx[609]) * xx[18] + xx[625] + xx
               [446]);
  J[2607] = (xx[532] * state[126] * state[127] + xx[628]) * xx[18] - xx[497];
  J[2608] = xx[18] * (xx[631] + xx[535] * state[126]) - xx[606];
  J[2609] = (xx[612] * state[126] + xx[637]) * xx[18];
  J[2652] = - ((xx[494] * xx[533] + xx[641]) * xx[18] + (xx[494] * xx[495] + xx
    [644]) * xx[18]);
  J[2653] = (xx[604] * xx[34] + xx[638]) * xx[18] + xx[380];
  J[2654] = motionData[125] + xx[18] * (xx[650] - xx[613] * xx[34]) + xx[399] +
    xx[361];
  J[2655] = xx[18] * (xx[653] - xx[68] * xx[34]) + xx[349] - motionData[124] +
    xx[363];
  J[2656] = xx[659];
  J[2657] = xx[662];
  J[2658] = xx[353];
  J[2710] = - (xx[18] * (xx[642] + xx[2] * xx[494]) + xx[18] * (xx[645] + xx[496]
    * xx[494]) - 8.752401783061839e-3);
  J[2711] = xx[18] * (xx[639] - xx[605] * xx[34]) + xx[435] - motionData[125] -
    xx[361];
  J[2712] = (xx[35] * xx[34] + xx[651]) * xx[18] + xx[359];
  J[2713] = motionData[123] + xx[18] * (xx[654] - xx[633] * xx[34]) + xx[465] -
    xx[389];
  J[2714] = xx[660];
  J[2715] = xx[663];
  J[2716] = xx[354];
  J[2768] = - ((xx[643] - xx[485] * xx[494]) * xx[18] + (xx[646] - xx[504] * xx
    [494]) * xx[18] - 0.1147611043125146);
  J[2769] = motionData[124] + xx[18] * (xx[640] - xx[607] * xx[34]) + xx[364] -
    xx[363];
  J[2770] = xx[18] * (xx[652] - xx[534] * xx[34]) + xx[360] - motionData[123] +
    xx[389];
  J[2771] = (xx[144] * xx[34] + xx[655]) * xx[18] + xx[348];
  J[2772] = xx[661];
  J[2773] = xx[664];
  J[2774] = xx[355];
  return 48;
}

static int isInKinematicSingularity_0(const RuntimeDerivedValuesBundle *rtdv,
  const int *modeVector, const double *motionData)
{
  const double *rtdvd = rtdv->mDoubles.mValues;
  const int *rtdvi = rtdv->mInts.mValues;
  (void) rtdvd;
  (void) rtdvi;
  (void) modeVector;
  (void) motionData;
  return 0;
}

static int isInKinematicSingularity_1(const RuntimeDerivedValuesBundle *rtdv,
  const int *modeVector, const double *motionData)
{
  const double *rtdvd = rtdv->mDoubles.mValues;
  const int *rtdvi = rtdv->mInts.mValues;
  (void) rtdvd;
  (void) rtdvi;
  (void) modeVector;
  (void) motionData;
  return 0;
}

static int isInKinematicSingularity_2(const RuntimeDerivedValuesBundle *rtdv,
  const int *modeVector, const double *motionData)
{
  const double *rtdvd = rtdv->mDoubles.mValues;
  const int *rtdvi = rtdv->mInts.mValues;
  (void) rtdvd;
  (void) rtdvi;
  (void) modeVector;
  (void) motionData;
  return 0;
}

static int isInKinematicSingularity_3(const RuntimeDerivedValuesBundle *rtdv,
  const int *modeVector, const double *motionData)
{
  const double *rtdvd = rtdv->mDoubles.mValues;
  const int *rtdvi = rtdv->mInts.mValues;
  (void) rtdvd;
  (void) rtdvi;
  (void) modeVector;
  (void) motionData;
  return 0;
}

static int isInKinematicSingularity_4(const RuntimeDerivedValuesBundle *rtdv,
  const int *modeVector, const double *motionData)
{
  const double *rtdvd = rtdv->mDoubles.mValues;
  const int *rtdvi = rtdv->mInts.mValues;
  (void) rtdvd;
  (void) rtdvi;
  (void) modeVector;
  (void) motionData;
  return 0;
}

static int isInKinematicSingularity_5(const RuntimeDerivedValuesBundle *rtdv,
  const int *modeVector, const double *motionData)
{
  const double *rtdvd = rtdv->mDoubles.mValues;
  const int *rtdvi = rtdv->mInts.mValues;
  (void) rtdvd;
  (void) rtdvi;
  (void) modeVector;
  (void) motionData;
  return 0;
}

static int isInKinematicSingularity_6(const RuntimeDerivedValuesBundle *rtdv,
  const int *modeVector, const double *motionData)
{
  const double *rtdvd = rtdv->mDoubles.mValues;
  const int *rtdvi = rtdv->mInts.mValues;
  (void) rtdvd;
  (void) rtdvi;
  (void) modeVector;
  (void) motionData;
  return 0;
}

static int isInKinematicSingularity_7(const RuntimeDerivedValuesBundle *rtdv,
  const int *modeVector, const double *motionData)
{
  const double *rtdvd = rtdv->mDoubles.mValues;
  const int *rtdvi = rtdv->mInts.mValues;
  (void) rtdvd;
  (void) rtdvi;
  (void) modeVector;
  (void) motionData;
  return 0;
}

static int isInKinematicSingularity_8(const RuntimeDerivedValuesBundle *rtdv,
  const int *modeVector, const double *motionData)
{
  const double *rtdvd = rtdv->mDoubles.mValues;
  const int *rtdvi = rtdv->mInts.mValues;
  (void) rtdvd;
  (void) rtdvi;
  (void) modeVector;
  (void) motionData;
  return 0;
}

static int isInKinematicSingularity_9(const RuntimeDerivedValuesBundle *rtdv,
  const int *modeVector, const double *motionData)
{
  const double *rtdvd = rtdv->mDoubles.mValues;
  const int *rtdvi = rtdv->mInts.mValues;
  (void) rtdvd;
  (void) rtdvi;
  (void) modeVector;
  (void) motionData;
  return 0;
}

static int isInKinematicSingularity_10(const RuntimeDerivedValuesBundle *rtdv,
  const int *modeVector, const double *motionData)
{
  const double *rtdvd = rtdv->mDoubles.mValues;
  const int *rtdvi = rtdv->mInts.mValues;
  (void) rtdvd;
  (void) rtdvi;
  (void) modeVector;
  (void) motionData;
  return 0;
}

static int isInKinematicSingularity_11(const RuntimeDerivedValuesBundle *rtdv,
  const int *modeVector, const double *motionData)
{
  const double *rtdvd = rtdv->mDoubles.mValues;
  const int *rtdvi = rtdv->mInts.mValues;
  (void) rtdvd;
  (void) rtdvi;
  (void) modeVector;
  (void) motionData;
  return 0;
}

static int isInKinematicSingularity_12(const RuntimeDerivedValuesBundle *rtdv,
  const int *modeVector, const double *motionData)
{
  const double *rtdvd = rtdv->mDoubles.mValues;
  const int *rtdvi = rtdv->mInts.mValues;
  (void) rtdvd;
  (void) rtdvi;
  (void) modeVector;
  (void) motionData;
  return 0;
}

static int isInKinematicSingularity_13(const RuntimeDerivedValuesBundle *rtdv,
  const int *modeVector, const double *motionData)
{
  const double *rtdvd = rtdv->mDoubles.mValues;
  const int *rtdvi = rtdv->mInts.mValues;
  (void) rtdvd;
  (void) rtdvi;
  (void) modeVector;
  (void) motionData;
  return 0;
}

static int isInKinematicSingularity_14(const RuntimeDerivedValuesBundle *rtdv,
  const int *modeVector, const double *motionData)
{
  const double *rtdvd = rtdv->mDoubles.mValues;
  const int *rtdvi = rtdv->mInts.mValues;
  (void) rtdvd;
  (void) rtdvi;
  (void) modeVector;
  (void) motionData;
  return 0;
}

static int isInKinematicSingularity_15(const RuntimeDerivedValuesBundle *rtdv,
  const int *modeVector, const double *motionData)
{
  const double *rtdvd = rtdv->mDoubles.mValues;
  const int *rtdvi = rtdv->mInts.mValues;
  (void) rtdvd;
  (void) rtdvi;
  (void) modeVector;
  (void) motionData;
  return 0;
}

static int isInKinematicSingularity_16(const RuntimeDerivedValuesBundle *rtdv,
  const int *modeVector, const double *motionData)
{
  const double *rtdvd = rtdv->mDoubles.mValues;
  const int *rtdvi = rtdv->mInts.mValues;
  (void) rtdvd;
  (void) rtdvi;
  (void) modeVector;
  (void) motionData;
  return 0;
}

static int isInKinematicSingularity_17(const RuntimeDerivedValuesBundle *rtdv,
  const int *modeVector, const double *motionData)
{
  const double *rtdvd = rtdv->mDoubles.mValues;
  const int *rtdvi = rtdv->mInts.mValues;
  (void) rtdvd;
  (void) rtdvi;
  (void) modeVector;
  (void) motionData;
  return 0;
}

static int isInKinematicSingularity_18(const RuntimeDerivedValuesBundle *rtdv,
  const int *modeVector, const double *motionData)
{
  const double *rtdvd = rtdv->mDoubles.mValues;
  const int *rtdvi = rtdv->mInts.mValues;
  (void) rtdvd;
  (void) rtdvi;
  (void) modeVector;
  (void) motionData;
  return 0;
}

int dempsystest_bjorn_59d2bdba_5_isInKinematicSingularity(const void *mech,
  const RuntimeDerivedValuesBundle *rtdv, size_t constraintIdx, const int
  *modeVector, const double *motionData)
{
  (void) mech;
  (void) rtdv;
  (void) modeVector;
  (void) motionData;
  switch (constraintIdx)
  {
   case 0:
    return isInKinematicSingularity_0(rtdv, modeVector, motionData);

   case 1:
    return isInKinematicSingularity_1(rtdv, modeVector, motionData);

   case 2:
    return isInKinematicSingularity_2(rtdv, modeVector, motionData);

   case 3:
    return isInKinematicSingularity_3(rtdv, modeVector, motionData);

   case 4:
    return isInKinematicSingularity_4(rtdv, modeVector, motionData);

   case 5:
    return isInKinematicSingularity_5(rtdv, modeVector, motionData);

   case 6:
    return isInKinematicSingularity_6(rtdv, modeVector, motionData);

   case 7:
    return isInKinematicSingularity_7(rtdv, modeVector, motionData);

   case 8:
    return isInKinematicSingularity_8(rtdv, modeVector, motionData);

   case 9:
    return isInKinematicSingularity_9(rtdv, modeVector, motionData);

   case 10:
    return isInKinematicSingularity_10(rtdv, modeVector, motionData);

   case 11:
    return isInKinematicSingularity_11(rtdv, modeVector, motionData);

   case 12:
    return isInKinematicSingularity_12(rtdv, modeVector, motionData);

   case 13:
    return isInKinematicSingularity_13(rtdv, modeVector, motionData);

   case 14:
    return isInKinematicSingularity_14(rtdv, modeVector, motionData);

   case 15:
    return isInKinematicSingularity_15(rtdv, modeVector, motionData);

   case 16:
    return isInKinematicSingularity_16(rtdv, modeVector, motionData);

   case 17:
    return isInKinematicSingularity_17(rtdv, modeVector, motionData);

   case 18:
    return isInKinematicSingularity_18(rtdv, modeVector, motionData);
  }

  return 0;
}

void dempsystest_bjorn_59d2bdba_5_convertStateVector(const void *asmMech, const
  RuntimeDerivedValuesBundle *rtdv, const void *simMech, const double *asmState,
  const int *asmModeVector, const int *simModeVector, double *simState)
{
  const double *rtdvd = rtdv->mDoubles.mValues;
  const int *rtdvi = rtdv->mInts.mValues;
  (void) asmMech;
  (void) rtdvd;
  (void) rtdvi;
  (void) simMech;
  (void) asmModeVector;
  (void) simModeVector;
  simState[0] = asmState[0];
  simState[1] = asmState[1];
  simState[2] = asmState[2];
  simState[3] = asmState[3];
  simState[4] = asmState[4];
  simState[5] = asmState[5];
  simState[6] = asmState[6];
  simState[7] = asmState[7];
  simState[8] = asmState[8];
  simState[9] = asmState[9];
  simState[10] = asmState[10];
  simState[11] = asmState[11];
  simState[12] = asmState[12];
  simState[13] = asmState[13];
  simState[14] = asmState[14];
  simState[15] = asmState[15];
  simState[16] = asmState[16];
  simState[17] = asmState[17];
  simState[18] = asmState[18];
  simState[19] = asmState[19];
  simState[20] = asmState[20];
  simState[21] = asmState[21];
  simState[22] = asmState[22];
  simState[23] = asmState[23];
  simState[24] = asmState[24];
  simState[25] = asmState[25];
  simState[26] = asmState[26];
  simState[27] = asmState[27];
  simState[28] = asmState[28];
  simState[29] = asmState[29];
  simState[30] = asmState[30];
  simState[31] = asmState[31];
  simState[32] = asmState[32];
  simState[33] = asmState[33];
  simState[34] = asmState[34];
  simState[35] = asmState[35];
  simState[36] = asmState[36];
  simState[37] = asmState[37];
  simState[38] = asmState[38];
  simState[39] = asmState[39];
  simState[40] = asmState[40];
  simState[41] = asmState[41];
  simState[42] = asmState[42];
  simState[43] = asmState[43];
  simState[44] = asmState[44];
  simState[45] = asmState[45];
  simState[46] = asmState[46];
  simState[47] = asmState[47];
  simState[48] = asmState[48];
  simState[49] = asmState[49];
  simState[50] = asmState[50];
  simState[51] = asmState[51];
  simState[52] = asmState[52];
  simState[53] = asmState[53];
  simState[54] = asmState[54];
  simState[55] = asmState[55];
  simState[56] = asmState[56];
  simState[57] = asmState[57];
  simState[58] = asmState[58];
  simState[59] = asmState[59];
  simState[60] = asmState[60];
  simState[61] = asmState[61];
  simState[62] = asmState[62];
  simState[63] = asmState[63];
  simState[64] = asmState[64];
  simState[65] = asmState[65];
  simState[66] = asmState[66];
  simState[67] = asmState[67];
  simState[68] = asmState[68];
  simState[69] = asmState[69];
  simState[70] = asmState[70];
  simState[71] = asmState[71];
  simState[72] = asmState[72];
  simState[73] = asmState[73];
  simState[74] = asmState[74];
  simState[75] = asmState[75];
  simState[76] = asmState[76];
  simState[77] = asmState[77];
  simState[78] = asmState[78];
  simState[79] = asmState[79];
  simState[80] = asmState[80];
  simState[81] = asmState[81];
  simState[82] = asmState[82];
  simState[83] = asmState[83];
  simState[84] = asmState[84];
  simState[85] = asmState[85];
  simState[86] = asmState[86];
  simState[87] = asmState[87];
  simState[88] = asmState[88];
  simState[89] = asmState[89];
  simState[90] = asmState[90];
  simState[91] = asmState[91];
  simState[92] = asmState[92];
  simState[93] = asmState[93];
  simState[94] = asmState[94];
  simState[95] = asmState[95];
  simState[96] = asmState[96];
  simState[97] = asmState[97];
  simState[98] = asmState[98];
  simState[99] = asmState[99];
  simState[100] = asmState[100];
  simState[101] = asmState[101];
  simState[102] = asmState[102];
  simState[103] = asmState[103];
  simState[104] = asmState[104];
  simState[105] = asmState[105];
  simState[106] = asmState[106];
  simState[107] = asmState[107];
  simState[108] = asmState[108];
  simState[109] = asmState[109];
  simState[110] = asmState[110];
  simState[111] = asmState[111];
  simState[112] = asmState[112];
  simState[113] = asmState[113];
  simState[114] = asmState[114];
  simState[115] = asmState[115];
  simState[116] = asmState[116];
  simState[117] = asmState[117];
  simState[118] = asmState[118];
  simState[119] = asmState[119];
  simState[120] = asmState[120];
  simState[121] = asmState[121];
  simState[122] = asmState[122];
  simState[123] = asmState[123];
  simState[124] = asmState[124];
  simState[125] = asmState[125];
  simState[126] = asmState[126];
  simState[127] = asmState[127];
  simState[128] = asmState[128];
  simState[129] = asmState[129];
  simState[130] = asmState[130];
  simState[131] = asmState[131];
  simState[132] = asmState[132];
}
