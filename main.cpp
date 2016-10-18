#include <stdio.h>
#include <stdint.h>

#define BIG_ENDIAN	// Define if host CPU is Big Endian

typedef bool pin;

typedef uint8_t Byte;
typedef uint16_t Word;
typedef uint8_t Reg8;
typedef uint16_t Reg16;
typedef uint32_t Reg32;

#define RAMSIZE 65536

Byte Ram[RAMSIZE];	// RAN Memory

pin Clock = 1;
pin nReset = 1;

Byte Instr;	// Current (fetched) instruction
Byte M;		// Current (fetched) memory byte

// Instruction Decoding    x x y y y z z z
//                         - - p p q - - -
//
#define GetX(i) ((i & 0b11000000) >> 6)
#define GetY(i) ((i & 0b00111000) >> 3)
#define GetZ(i)  (i & 0b00000111)
#define GetP(i)	((i & 0b00110000) >> 4)
#define GetQ(i) ((i & 0b00001000) >> 3)

// Register table
// 0  1  2  3  4  5  6   7   8   9   10 12 14 15 16 17 18 19 20 21 22 23 24 25
// B  C  D  E  H  L  IXh IXl IYh IYl SP PC A  F  B' C' D' E' H' L' A' F' I  R
Reg8 Regs[26];

#ifdef BIG_ENDIAN
//Pointers to the 8 bit registers
Reg8 *pB = &Regs[1];
Reg8 *pC = &Regs[0];
Reg8 *pD = &Regs[3];
Reg8 *pE = &Regs[2];
Reg8 *pH = &Regs[5];
Reg8 *pL = &Regs[4];
Reg8 *pIXh = &Regs[7];
Reg8 *pIXl = &Regs[6];
Reg8 *pIYh = &Regs[9];
Reg8 *pIYl = &Regs[8];
Reg8 *pA = &Regs[15];
Reg8 *pF = &Regs[14];
Reg8 *pB_ = &Regs[17];
Reg8 *pC_ = &Regs[16];
Reg8 *pD_ = &Regs[19];
Reg8 *pE_ = &Regs[18];
Reg8 *pH_ = &Regs[21];
Reg8 *pL_ = &Regs[20];
Reg8 *pA_ = &Regs[23];
Reg8 *pF_ = &Regs[22];
Reg8 *pI = &Regs[25];
Reg8 *pR = &Regs[24];
#else
Reg8 *pB = &Regs[0];
Reg8 *pC = &Regs[1];
Reg8 *pD = &Regs[2];
Reg8 *pE = &Regs[3];
Reg8 *pH = &Regs[4];
Reg8 *pL = &Regs[5];
Reg8 *pIXh = &Regs[6];
Reg8 *pIXl = &Regs[7];
Reg8 *pIYh = &Regs[8];
Reg8 *pIYl = &Regs[9];
Reg8 *pA = &Regs[14];
Reg8 *pF = &Regs[15];
Reg8 *pB_ = &Regs[16];
Reg8 *pC_ = &Regs[17];
Reg8 *pD_ = &Regs[18];
Reg8 *pE_ = &Regs[19];
Reg8 *pH_ = &Regs[20];
Reg8 *pL_ = &Regs[21];
Reg8 *pA_ = &Regs[22];
Reg8 *pF_ = &Regs[23];
Reg8 *pI = &Regs[24];
Reg8 *pR = &Regs[25];
#endif

// Pointers to the 16 bit registers
Reg16 *pBC = (Reg16*)&Regs[0];
Reg16 *pDE = (Reg16*)&Regs[2];
Reg16 *pHL = (Reg16*)&Regs[4];
Reg16 *pIX = (Reg16*)&Regs[6];
Reg16 *pIY = (Reg16*)&Regs[8];
Reg16 *pSP = (Reg16*)&Regs[10];
Reg16 *pPC = (Reg16*)&Regs[12];
Reg16 *pAF = (Reg16*)&Regs[14];
Reg16 *pBC_ = (Reg16*)&Regs[16];
Reg16 *pDE_ = (Reg16*)&Regs[18];
Reg16 *pHL_ = (Reg16*)&Regs[20];
Reg16 *pAF_ = (Reg16*)&Regs[22];

// 8 Bit registers
#define B (*pB)
#define C (*pC)
#define D (*pD)
#define E (*pE)
#define H (*pH)
#define L (*pL)
#define IXh (*pIXh)
#define IXl (*pIXl)
#define IYh (*pIYh)
#define IYl (*pIYl)
#define A (*pA)
#define F (*pF)
#define B_ (*pB_)
#define C_ (*pC_)
#define D_ (*pD_)
#define E_ (*pE_)
#define H_ (*pH_)
#define L_ (*pL_)
#define A_ (*pA_)
#define F_ (*pF_)
#define I (*pI)
#define R (*pR)

// 16 Bit registers
#define BC (*pBC)
#define DE (*pDE)
#define HL (*pHL)
#define IX (*pIX)
#define IY (*pIY)
#define SP (*pSP)
#define PC (*pPC)
#define AF (*pAF)
#define BC_ (*pBC_)
#define DE_ (*pDE_)
#define HL_ (*pHL_)
#define AF_ (*pAF_)

// Flags          (SZ5H3PNC)
#define fS	(F & 0b10000000)
#define fZ	(F & 0b01000000)
#define	f5	(F & 0b00100000)
#define fH	(F & 0b00010000)
#define f3	(F & 0b00001000)
#define fP	(F & 0b00000100)
#define fN	(F & 0b00000010)
#define fC	(F & 0b00000001)

// Data tables
Reg16 *rp[4];	// BC DE HL SP
Reg16 *rp2[4];	// BC DE HL AF
Reg8 *r[8];		// B C D E H L (HL) A

// Table for quick calculation of the parity flag
static const Byte parityTable[256] = {
	4,0,0,4,0,4,4,0,0,4,4,0,4,0,0,4,
	0,4,4,0,4,0,0,4,4,0,0,4,0,4,4,0,
	0,4,4,0,4,0,0,4,4,0,0,4,0,4,4,0,
	4,0,0,4,0,4,4,0,0,4,4,0,4,0,0,4,
	0,4,4,0,4,0,0,4,4,0,0,4,0,4,4,0,
	4,0,0,4,0,4,4,0,0,4,4,0,4,0,0,4,
	4,0,0,4,0,4,4,0,0,4,4,0,4,0,0,4,
	0,4,4,0,4,0,0,4,4,0,0,4,0,4,4,0,
	0,4,4,0,4,0,0,4,4,0,0,4,0,4,4,0,
	4,0,0,4,0,4,4,0,0,4,4,0,4,0,0,4,
	4,0,0,4,0,4,4,0,0,4,4,0,4,0,0,4,
	0,4,4,0,4,0,0,4,4,0,0,4,0,4,4,0,
	4,0,0,4,0,4,4,0,0,4,4,0,4,0,0,4,
	0,4,4,0,4,0,0,4,4,0,0,4,0,4,4,0,
	0,4,4,0,4,0,0,4,4,0,0,4,0,4,4,0,
	4,0,0,4,0,4,4,0,0,4,4,0,4,0,0,4,
};

// Table for getting the flags of 16 bits math
static const Byte cbitsTable[512] = {
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,
	1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
	17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,
	1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
	17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,
	1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
	17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,
	1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
	17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,
	1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
	17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,
	1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
	17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,
	1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
	17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,
	1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
	17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,
};

// Memory manipulation functions
// (Must be different for hardware based emulation)
Byte Fetch8(Word Addr) {
	return(Ram[Addr]);
}

void Write8(Word Addr, Byte Value) {
	Ram[Addr] = Value;
}

Word Fetch16(Word Addr) {
#ifdef BIG_ENDIAN
	return((Ram[Addr + 1] << 8) + Ram[Addr]);
#else
	return((Ram[Addr] << 8) + Ram[Addr + 1]);
#endif
}

void Write16(Word Addr, Word Value) {
#ifdef BIG_ENDIAN
	Ram[Addr + 1] = (Value & 0xff);
	Ram[Addr] = ((Value >> 8) & 0xff);
#else
	Ram[Addr] = (Value & 0xff);
	Ram[Addr + 1] = ((Value >> 8) & 0xff);
#endif
}

void TestLoad(void) {	// Loads the memory with a small test application
	int i = 0;

//	Ram[i++] = 0x00;	// NOP
	Ram[i++] = 0x01;	// LD BC, 0x0F29
	Ram[i++] = 0x29;
	Ram[i++] = 0x0F;
//	Ram[i++] = 0x02;	// LD (BC), A
//	Ram[i++] = 0x03;	// INC BC
//	Ram[i++] = 0x04;	// INC B
//	Ram[i++] = 0x05;	// DEC B
//	Ram[i++] = 0x06;	// LD B, 0x7F
//	Ram[i++] = 0x7F;
//	Ram[i++] = 0x06;	// LD B, 0x00
//	Ram[i++] = 0x00;
//	Ram[i++] = 0x0E;	// LD C, 0x03
//	Ram[i++] = 0x03;
//	Ram[i++] = 0x08;	// EX AF, AF'
//	Ram[i++] = 0x10;	// DJNZ, -2
//	Ram[i++] = 0xFE;
	Ram[i++] = 0x21;	// LD HL, 0x0200
	Ram[i++] = 0x00;
	Ram[i++] = 0x02;
	Ram[i++] = 0x09;	// ADD HL, BC
	Ram[i++] = 0x76;	// HLT

}

void Z80Init(void) {	// Must be called before anything else

	// Prepare register pair table
	rp[0] = pBC;
	rp[1] = pDE;
	rp[2] = pHL;
	rp[3] = pSP;

	// Prepare second register pair table
	rp2[0] = pBC;
	rp2[1] = pDE;
	rp2[2] = pHL;
	rp2[3] = pAF;

	// Prepare register table
	r[0] = pB;
	r[1] = pC;
	r[2] = pD;
	r[3] = pE;
	r[4] = pH;
	r[5] = pL;
	r[7] = pA;

}

void Z80Reset(void) {	// Resets all Z80 registers (and other stuff) to their initial values
	Byte i;
	for (i = 0; i < 26; i++) {
		Regs[i] = 0;
	}
}

Byte Z80DecodeCB(void) {	// Decodes and executes the CB prefix instruction
	Byte Result = 0;
	Byte x, y, z, p, q;
	Reg8 Tmp8;
	Reg16 Tmp16;

	Instr = Fetch8(PC++);
	x = GetX(Instr);
	y = GetY(Instr);
	z = GetZ(Instr);
	p = GetP(Instr);
	q = GetQ(Instr);

	if (x == 0) {						// rot[y] r[z]

	}
	if (x == 1) {						// BIT y, r[z]

	}
	if (x == 2) {						// RES y, r[z]

	}
	if (x == 3) {						// SET y, r[z]

	}
	return(Result);
}

Byte Z80DecodeDD(void) {	// Decodes and executes the CB prefix instruction
	Byte Result = 0;
	Byte x, y, z, p, q;
	Reg8 Tmp8;
	Reg16 Tmp16;

	Instr = Fetch8(PC++);
	x = GetX(Instr);
	y = GetY(Instr);
	z = GetZ(Instr);
	p = GetP(Instr);
	q = GetQ(Instr);

	return(Result);
}

Byte Z80DecodeED(void) {	// Decodes and executes the CB prefix instruction
	Byte Result = 0;
	Byte x, y, z, p, q;
	Reg8 Tmp8;
	Reg16 Tmp16;

	Instr = Fetch8(PC++);
	x = GetX(Instr);
	y = GetY(Instr);
	z = GetZ(Instr);
	p = GetP(Instr);
	q = GetQ(Instr);

	return(Result);
}

Byte Z80DecodeFD(void) {	// Decodes and executes the CB prefix instruction
	Byte Result = 0;
	Byte x, y, z, p, q;
	Reg8 Tmp8;
	Reg16 Tmp16;

	Instr = Fetch8(PC++);
	x = GetX(Instr);
	y = GetY(Instr);
	z = GetZ(Instr);
	p = GetP(Instr);
	q = GetQ(Instr);

	return(Result);
}

Byte Z80Decode(void) {		// Decodes and executes the instruction pointed at by PC
	Byte Result = 0;
	Byte x, y, z, p, q;
	Reg8 Tmp8;
	Reg16 Tmp16;
	Reg32 Tmp32;

	if (Clock) {
		Instr = Fetch8(PC++);
		x = GetX(Instr);
		y = GetY(Instr);
		z = GetZ(Instr);
		p = GetP(Instr);
		q = GetQ(Instr);

		if (x == 0) {
			if (z == 0) {
				if (y == 0) {			// NOP
					// Do nothing
				}
				if (y == 1) {			// EX AF,AF'
					Tmp16 = AF; AF = AF_; AF_ = Tmp16;
				}
				if (y == 2) {			// DJNZ d
					Tmp8 = Fetch8(PC++);
					if (--BC) {
						PC--;
						PC += (int8_t)Tmp8;
					}
				}
				if (y == 3) {			// JR d
					Tmp8 = Fetch8(PC++);
					PC += (int8_t)Tmp8;
				}
				if (y > 3) {			// JR cc[y-4], d
					Tmp8 = Fetch8(PC++);
					if (y == 4) {		// JR NZ, d
						if (!fZ)
							PC += (int8_t)Tmp8;
					}
					if (y == 5) {		// JR Z, d
						if (fZ)
							PC += (int8_t)Tmp8;
					}
					if (y == 6) {		// JR NC, d
						if (!fC)
							PC += (int8_t)Tmp8;
					}
					if (y == 7) {		// JR C, d
						if (fC)
							PC += (int8_t)Tmp8;
					}
				}
			}
			if (z == 1) {
				if (q == 0) {			// LD rp[p], nn
					Tmp16 = Fetch16(PC); PC += 2;
					*rp[p] = Tmp16;
				}
				if (q == 1) {			// ADD HL, rp[p]
					Tmp32 = HL + *rp[p];
					F = (F & ~0x3b) | ((Tmp32 >> 8) & 0x28) | cbitsTable[(HL ^ *rp[p] ^ Tmp32) >> 8];
					HL = Tmp32;
				}
			}
			if (z == 2) {
				if (q == 0) {
					if (p == 0) {		// LD (BC), A
						Write8(BC, A);
					}
					if (p == 1) {		// LD (DE), A
						Write8(DE, A);
					}
					if (p == 2) {		// LD (nn), HL
						Tmp16 = Fetch16(PC); PC += 2;
						Write16(Tmp16, HL);
					}
					if (p == 3) {		// LD (nn), A
						Tmp16 = Fetch16(PC); PC += 2;
						Write8(Tmp16, A);
					}
				}
				if (q == 1) {
					if (p == 0) {		// LD A, (BC)
						A = Fetch8(BC);
					}
					if (p == 1) {		// LD A, (DE)
						A = Fetch8(DE);
					}
					if (p == 2) {		// LD HL, (nn)
						Tmp16 = Fetch16(PC); PC += 2;
						HL = Fetch16(Tmp16);
					}
					if (p == 3) {		// LD A, (nn)
						Tmp16 = Fetch16(PC); PC += 2;
						A = Fetch8(Tmp16);
					}
				}
			}
			if (z == 3) {
				if (q == 0) {			// INC rp[p]
					(*rp[p])++;
				}
				if (q == 1) {			// DEC rp[p]
					(*rp[p])--;
				}
			}
			if (z == 4) {				// INC r[y]
				if (y == 6) {
					Tmp8 = Fetch8(HL);
					Tmp8++;
					Write8(HL, Tmp8);
				} else {
					(*r[y])++;
				}
			}
			if (z == 5) {				// DEC r[y]
				if (y == 6) {
					Tmp8 = Fetch8(HL);
					Tmp8--;
					Write8(HL, Tmp8);
				} else {
					(*r[y])--;
				}
			}
			if (z == 6) {				// LD r[y], n
				Tmp8 = Fetch8(PC++);
				if (y == 6) {
					Write8(HL, Tmp8);
				} else {
					(*r[y]) = Tmp8;
				}
			}
			if (z == 7) {
				if (y == 0) {			// RLCA

				}
				if (y == 1) {			// RRCA

				}
				if (y == 2) {			// RLA

				}
				if (y == 3) {			// RRA

				}
				if (y == 4) {			// DAA

				}
				if (y == 5) {			// CPL

				}
				if (y == 6) {			// SCF

				}
				if (y == 7) {			// CCF

				}
			}
		}
		if (x == 1) {
			if (z == 6) {
				if (y == 6) {			// HLT (replaces LD (HL), (HL))
					Result = 1;
				} else {				// LD r[y], (HL)
					Tmp8 = Fetch8(HL);
					(*r[y]) = Tmp8;
				}
			} else {
				if (y == 6) {			// LD (HL), r[z]
					Write8(HL, (*r[z]));
				} else {				// LD r[y], r[z]
					(*r[y]) = (*r[z]);
				}
			}
		}
		if (x == 2) {					// alu[y] r[z]
			if (y == 0) {				// ADD A, r[z]

			}
			if (y == 1) {				// ADC A, r[z]

			}
			if (y == 2) {				// SUB r[z]

			}
			if (y == 3) {				// SBC A,

			}
			if (y == 4) {				// AND

			}
			if (y == 5) {				// XOR

			}
			if (y == 6) {				// OR

			}
			if (y == 7) {				// CP

			}
		}
		if (x == 3) {
			if (z == 0) {				// RET cc[y]

			}
			if (z == 1) {
				if (q == 0) {			// POP rp2[p]

				} else {
					if (p == 0) {		// RET

					}
					if (p == 1) {		// EXX

					}
					if (p == 2) {		// JP HL

					}
					if (p == 3) {		// LD SP,HL

					}
				}

			}
			if (z == 2) {				// JP cc[y], nn

			}
			if (z == 3) {
				if (y == 0) {			// JP nn

				}
				if (y == 1) {			// (CB prefix)
					Z80DecodeCB();
				}
				if (y == 2) {			// OUT (n), A

				}
				if (y == 3) {			// IN A, (n)

				}
				if (y == 4) {			// EX (SP), HL

				}
				if (y == 5) {			// EX DE, HL

				}
				if (y == 6) {			// DI

				}
				if (y == 7) {			// EI

				}
			}
			if (z == 4) {				// CALL cc[y], nn

			}
			if (z == 5) {
				if (q == 0) {			// PUSH rp2[p]

				} else {
					if (p == 0) {		// CALL nn

					}
					if (p == 1) {		// (DD prefix)
						Z80DecodeDD();
					}
					if (p == 2) {		// (ED prefix)
						Z80DecodeED();
					}
					if (p == 3) {		// (FD prefix)
						Z80DecodeFD();
					}
				}
				if (z == 6) {			// alu[y] n

				}
				if (z == 7) {			// RST y*8

				}
			}
		}
	}
	return(Result);
}

int main(void) {
	printf("MyZ80 Test Interface\n");
	printf("Copyright (C) 2016 Marcelo F. Dantas\n");

	Z80Init();

	TestLoad();

	Z80Reset();
	while (true) {
		if (Z80Decode()) {
			break;
		}
	}
}