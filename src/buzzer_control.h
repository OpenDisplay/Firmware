#ifndef BUZZER_CONTROL_H
#define BUZZER_CONTROL_H

#include <stdint.h>

// Human-readable note names for authoring melodies. Values are the protocol
// frequency indices carried in the 0x0077 payload, so enumerators drop straight
// into byte arrays: {nA4, 40, nNone, 10, nA5, 40}.
//
// Freq(idx) = 13.75 * 2^(idx/24)  -- 24 quarter-tones/octave, anchored at
// A-1 = 13.75 Hz. One index step = one quarter-tone (50 cents). idx 0 = nNone
// (silence/rest). Naming: n<Note><s?><octave><p?>  ('s' = sharp, trailing 'p' =
// plus one quarter-tone). Octaves increment at C; C0 = idx 6, 24 indices per
// octave; idx 1..5 are octave -1 ("m1"). 12-TET notes land on even indices
// (nA4 = 120 = 440.00 Hz exact, nC5 = 126, nG8 = 212).
//
// Generated -- do not hand-edit. Regenerate the enum body with:
//   python3 - <<'EOF'
//   S=["C","Cs","D","Ds","E","F","Fs","G","Gs","A","As","B"]
//   def n(i):
//       if i==0: return "nNone"
//       r=i-6; o=r//24; f=r%24
//       oc=("m%d"%-o) if o<0 else "%d"%o
//       return "n%s%s%s"%(S[f//2],oc,"p" if f&1 else "")
//   print(",".join(n(i) for i in range(256)))
//   EOF
enum BuzzerNote : uint8_t {
    nNone    =   0, nAm1p    =   1, nAsm1    =   2, nAsm1p   =   3,
    nBm1     =   4, nBm1p    =   5, nC0      =   6, nC0p     =   7,
    nCs0     =   8, nCs0p    =   9, nD0      =  10, nD0p     =  11,
    nDs0     =  12, nDs0p    =  13, nE0      =  14, nE0p     =  15,
    nF0      =  16, nF0p     =  17, nFs0     =  18, nFs0p    =  19,
    nG0      =  20, nG0p     =  21, nGs0     =  22, nGs0p    =  23,
    nA0      =  24, nA0p     =  25, nAs0     =  26, nAs0p    =  27,
    nB0      =  28, nB0p     =  29, nC1      =  30, nC1p     =  31,
    nCs1     =  32, nCs1p    =  33, nD1      =  34, nD1p     =  35,
    nDs1     =  36, nDs1p    =  37, nE1      =  38, nE1p     =  39,
    nF1      =  40, nF1p     =  41, nFs1     =  42, nFs1p    =  43,
    nG1      =  44, nG1p     =  45, nGs1     =  46, nGs1p    =  47,
    nA1      =  48, nA1p     =  49, nAs1     =  50, nAs1p    =  51,
    nB1      =  52, nB1p     =  53, nC2      =  54, nC2p     =  55,
    nCs2     =  56, nCs2p    =  57, nD2      =  58, nD2p     =  59,
    nDs2     =  60, nDs2p    =  61, nE2      =  62, nE2p     =  63,
    nF2      =  64, nF2p     =  65, nFs2     =  66, nFs2p    =  67,
    nG2      =  68, nG2p     =  69, nGs2     =  70, nGs2p    =  71,
    nA2      =  72, nA2p     =  73, nAs2     =  74, nAs2p    =  75,
    nB2      =  76, nB2p     =  77, nC3      =  78, nC3p     =  79,
    nCs3     =  80, nCs3p    =  81, nD3      =  82, nD3p     =  83,
    nDs3     =  84, nDs3p    =  85, nE3      =  86, nE3p     =  87,
    nF3      =  88, nF3p     =  89, nFs3     =  90, nFs3p    =  91,
    nG3      =  92, nG3p     =  93, nGs3     =  94, nGs3p    =  95,
    nA3      =  96, nA3p     =  97, nAs3     =  98, nAs3p    =  99,
    nB3      = 100, nB3p     = 101, nC4      = 102, nC4p     = 103,
    nCs4     = 104, nCs4p    = 105, nD4      = 106, nD4p     = 107,
    nDs4     = 108, nDs4p    = 109, nE4      = 110, nE4p     = 111,
    nF4      = 112, nF4p     = 113, nFs4     = 114, nFs4p    = 115,
    nG4      = 116, nG4p     = 117, nGs4     = 118, nGs4p    = 119,
    nA4      = 120, nA4p     = 121, nAs4     = 122, nAs4p    = 123,
    nB4      = 124, nB4p     = 125, nC5      = 126, nC5p     = 127,
    nCs5     = 128, nCs5p    = 129, nD5      = 130, nD5p     = 131,
    nDs5     = 132, nDs5p    = 133, nE5      = 134, nE5p     = 135,
    nF5      = 136, nF5p     = 137, nFs5     = 138, nFs5p    = 139,
    nG5      = 140, nG5p     = 141, nGs5     = 142, nGs5p    = 143,
    nA5      = 144, nA5p     = 145, nAs5     = 146, nAs5p    = 147,
    nB5      = 148, nB5p     = 149, nC6      = 150, nC6p     = 151,
    nCs6     = 152, nCs6p    = 153, nD6      = 154, nD6p     = 155,
    nDs6     = 156, nDs6p    = 157, nE6      = 158, nE6p     = 159,
    nF6      = 160, nF6p     = 161, nFs6     = 162, nFs6p    = 163,
    nG6      = 164, nG6p     = 165, nGs6     = 166, nGs6p    = 167,
    nA6      = 168, nA6p     = 169, nAs6     = 170, nAs6p    = 171,
    nB6      = 172, nB6p     = 173, nC7      = 174, nC7p     = 175,
    nCs7     = 176, nCs7p    = 177, nD7      = 178, nD7p     = 179,
    nDs7     = 180, nDs7p    = 181, nE7      = 182, nE7p     = 183,
    nF7      = 184, nF7p     = 185, nFs7     = 186, nFs7p    = 187,
    nG7      = 188, nG7p     = 189, nGs7     = 190, nGs7p    = 191,
    nA7      = 192, nA7p     = 193, nAs7     = 194, nAs7p    = 195,
    nB7      = 196, nB7p     = 197, nC8      = 198, nC8p     = 199,
    nCs8     = 200, nCs8p    = 201, nD8      = 202, nD8p     = 203,
    nDs8     = 204, nDs8p    = 205, nE8      = 206, nE8p     = 207,
    nF8      = 208, nF8p     = 209, nFs8     = 210, nFs8p    = 211,
    nG8      = 212, nG8p     = 213, nGs8     = 214, nGs8p    = 215,
    nA8      = 216, nA8p     = 217, nAs8     = 218, nAs8p    = 219,
    nB8      = 220, nB8p     = 221, nC9      = 222, nC9p     = 223,
    nCs9     = 224, nCs9p    = 225, nD9      = 226, nD9p     = 227,
    nDs9     = 228, nDs9p    = 229, nE9      = 230, nE9p     = 231,
    nF9      = 232, nF9p     = 233, nFs9     = 234, nFs9p    = 235,
    nG9      = 236, nG9p     = 237, nGs9     = 238, nGs9p    = 239,
    nA9      = 240, nA9p     = 241, nAs9     = 242, nAs9p    = 243,
    nB9      = 244, nB9p     = 245, nC10     = 246, nC10p    = 247,
    nCs10    = 248, nCs10p   = 249, nD10     = 250, nD10p    = 251,
    nDs10    = 252, nDs10p   = 253, nE10     = 254, nE10p    = 255,
};

void initPassiveBuzzers(void);
void handleBuzzerActivate(uint8_t* data, uint16_t len);
void passiveBuzzerPowerOffAlert(void);
void buzzerService(void);   // non-blocking playback tick, called from loop()

#endif
