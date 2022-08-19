/* -*- Mode: C++; tab-width: 8; indent-tabs-mode: nil; c-basic-offset: 2 -*-
 * vim: set ts=8 sts=2 et sw=2 tw=80:
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */

#ifndef jit_riscv64_disasm_Constant_riscv64_h
#define jit_riscv64_disasm_Constant_riscv64_h
#  include "mozilla/Assertions.h"
#  include "mozilla/Types.h"

#  include <stdio.h>


namespace js {
namespace jit {
namespace disasm {
typedef unsigned char byte;

// A reasonable (ie, safe) buffer size for the disassembly of a single
// instruction.
const int ReasonableBufferSize = 256;

// Vector as used by the original code to allow for minimal modification.
// Functions exactly like a character array with helper methods.
template <typename T>
class V8Vector {
 public:
  V8Vector() : start_(nullptr), length_(0) {}
  V8Vector(T* data, int length) : start_(data), length_(length) {
    MOZ_ASSERT(length == 0 || (length > 0 && data != nullptr));
  }

  // Returns the length of the vector.
  int length() const { return length_; }

  // Returns the pointer to the start of the data in the vector.
  T* start() const { return start_; }

  // Access individual vector elements - checks bounds in debug mode.
  T& operator[](int index) const {
    MOZ_ASSERT(0 <= index && index < length_);
    return start_[index];
  }

  inline V8Vector<T> operator+(int offset) {
    MOZ_ASSERT(offset < length_);
    return V8Vector<T>(start_ + offset, length_ - offset);
  }

 private:
  T* start_;
  int length_;
};

template <typename T, int kSize>
class EmbeddedVector : public V8Vector<T> {
 public:
  EmbeddedVector() : V8Vector<T>(buffer_, kSize) {}

  explicit EmbeddedVector(T initial_value) : V8Vector<T>(buffer_, kSize) {
    for (int i = 0; i < kSize; ++i) {
      buffer_[i] = initial_value;
    }
  }

  // When copying, make underlying Vector to reference our buffer.
  EmbeddedVector(const EmbeddedVector& rhs) : V8Vector<T>(rhs) {
    MemCopy(buffer_, rhs.buffer_, sizeof(T) * kSize);
    this->set_start(buffer_);
  }

  EmbeddedVector& operator=(const EmbeddedVector& rhs) {
    if (this == &rhs) return *this;
    V8Vector<T>::operator=(rhs);
    MemCopy(buffer_, rhs.buffer_, sizeof(T) * kSize);
    this->set_start(buffer_);
    return *this;
  }

 private:
  T buffer_[kSize];
};

// // Helper function for printing to a Vector.
// static int MOZ_FORMAT_PRINTF(2, 3)
//     SNPrintF(V8Vector<char> str, const char* format, ...) {
//   va_list args;
//   va_start(args, format);
//   int result = vsnprintf(str.start(), str.length(), format, args);
//   va_end(args);
//   return result;
// }
}   // namespace disasm

// On RISCV all instructions are 32 bits, except for RVC.
using Instr = int32_t;
using ShortInstr = int16_t;

// ----- Fields offset and length.
// RISCV constants
const int kBaseOpcodeShift = 0;
const int kBaseOpcodeBits = 7;
const int kFunct7Shift = 25;
const int kFunct7Bits = 7;
const int kFunct5Shift = 27;
const int kFunct5Bits = 5;
const int kFunct3Shift = 12;
const int kFunct3Bits = 3;
const int kFunct2Shift = 25;
const int kFunct2Bits = 2;
const int kRs1Shift = 15;
const int kRs1Bits = 5;
const int kVs1Shift = 15;
const int kVs1Bits = 5;
const int kVs2Shift = 20;
const int kVs2Bits = 5;
const int kVdShift = 7;
const int kVdBits = 5;
const int kRs2Shift = 20;
const int kRs2Bits = 5;
const int kRs3Shift = 27;
const int kRs3Bits = 5;
const int kRdShift = 7;
const int kRdBits = 5;
const int kRlShift = 25;
const int kAqShift = 26;
const int kImm12Shift = 20;
const int kImm12Bits = 12;
const int kImm11Shift = 2;
const int kImm11Bits = 11;
const int kShamtShift = 20;
const int kShamtBits = 5;
const int kShamtWShift = 20;
// FIXME: remove this once we have a proper way to handle the wide shift amount
const int kShamtWBits = 6;
const int kArithShiftShift = 30;
const int kImm20Shift = 12;
const int kImm20Bits = 20;
const int kCsrShift = 20;
const int kCsrBits = 12;
const int kMemOrderBits = 4;
const int kPredOrderShift = 24;
const int kSuccOrderShift = 20;

// for C extension
const int kRvcFunct4Shift = 12;
const int kRvcFunct4Bits = 4;
const int kRvcFunct3Shift = 13;
const int kRvcFunct3Bits = 3;
const int kRvcRs1Shift = 7;
const int kRvcRs1Bits = 5;
const int kRvcRs2Shift = 2;
const int kRvcRs2Bits = 5;
const int kRvcRdShift = 7;
const int kRvcRdBits = 5;
const int kRvcRs1sShift = 7;
const int kRvcRs1sBits = 3;
const int kRvcRs2sShift = 2;
const int kRvcRs2sBits = 3;
const int kRvcFunct2Shift = 5;
const int kRvcFunct2BShift = 10;
const int kRvcFunct2Bits = 2;
const int kRvcFunct6Shift = 10;
const int kRvcFunct6Bits = 6;

const uint32_t kRvcOpcodeMask =
    0b11 | (((1 << kRvcFunct3Bits) - 1) << kRvcFunct3Shift);
const uint32_t kRvcFunct3Mask =
    (((1 << kRvcFunct3Bits) - 1) << kRvcFunct3Shift);
const uint32_t kRvcFunct4Mask =
    (((1 << kRvcFunct4Bits) - 1) << kRvcFunct4Shift);
const uint32_t kRvcFunct6Mask =
    (((1 << kRvcFunct6Bits) - 1) << kRvcFunct6Shift);
const uint32_t kRvcFunct2Mask =
    (((1 << kRvcFunct2Bits) - 1) << kRvcFunct2Shift);
const uint32_t kRvcFunct2BMask =
    (((1 << kRvcFunct2Bits) - 1) << kRvcFunct2BShift);
const uint32_t kCRTypeMask = kRvcOpcodeMask | kRvcFunct4Mask;
const uint32_t kCSTypeMask = kRvcOpcodeMask | kRvcFunct6Mask;
const uint32_t kCATypeMask = kRvcOpcodeMask | kRvcFunct6Mask | kRvcFunct2Mask;
const uint32_t kRvcBImm8Mask = (((1 << 5) - 1) << 2) | (((1 << 3) - 1) << 10);

// RISCV Instruction bit masks
const uint32_t kBaseOpcodeMask = ((1 << kBaseOpcodeBits) - 1)
                                 << kBaseOpcodeShift;
const uint32_t kFunct3Mask = ((1 << kFunct3Bits) - 1) << kFunct3Shift;
const uint32_t kFunct5Mask = ((1 << kFunct5Bits) - 1) << kFunct5Shift;
const uint32_t kFunct7Mask = ((1 << kFunct7Bits) - 1) << kFunct7Shift;
const uint32_t kFunct2Mask = 0b11 << kFunct7Shift;
const uint32_t kRTypeMask = kBaseOpcodeMask | kFunct3Mask | kFunct7Mask;
const uint32_t kRATypeMask = kBaseOpcodeMask | kFunct3Mask | kFunct5Mask;
const uint32_t kRFPTypeMask = kBaseOpcodeMask | kFunct7Mask;
const uint32_t kR4TypeMask = kBaseOpcodeMask | kFunct3Mask | kFunct2Mask;
const uint32_t kITypeMask = kBaseOpcodeMask | kFunct3Mask;
const uint32_t kSTypeMask = kBaseOpcodeMask | kFunct3Mask;
const uint32_t kBTypeMask = kBaseOpcodeMask | kFunct3Mask;
const uint32_t kUTypeMask = kBaseOpcodeMask;
const uint32_t kJTypeMask = kBaseOpcodeMask;
const uint32_t kRs1FieldMask = ((1 << kRs1Bits) - 1) << kRs1Shift;
const uint32_t kRs2FieldMask = ((1 << kRs2Bits) - 1) << kRs2Shift;
const uint32_t kRs3FieldMask = ((1 << kRs3Bits) - 1) << kRs3Shift;
const uint32_t kRdFieldMask = ((1 << kRdBits) - 1) << kRdShift;
const uint32_t kBImm12Mask = kFunct7Mask | kRdFieldMask;
const uint32_t kImm20Mask = ((1 << kImm20Bits) - 1) << kImm20Shift;
const uint32_t kImm12Mask = ((1 << kImm12Bits) - 1) << kImm12Shift;
const uint32_t kImm11Mask = ((1 << kImm11Bits) - 1) << kImm11Shift;
const uint32_t kImm31_12Mask = ((1 << 20) - 1) << 12;
const uint32_t kImm19_0Mask = ((1 << 20) - 1);

const int kNopByte = 0x00000013;

enum BaseOpcode : uint32_t {
  LOAD = 0b0000011,      // I form: LB LH LW LBU LHU
  LOAD_FP = 0b0000111,   // I form: FLW FLD FLQ
  MISC_MEM = 0b0001111,  // I special form: FENCE FENCE.I
  OP_IMM = 0b0010011,    // I form: ADDI SLTI SLTIU XORI ORI ANDI SLLI SRLI SRAI
  // Note: SLLI/SRLI/SRAI I form first, then func3 001/101 => R type
  AUIPC = 0b0010111,      // U form: AUIPC
  OP_IMM_32 = 0b0011011,  // I form: ADDIW SLLIW SRLIW SRAIW
  // Note:  SRLIW SRAIW I form first, then func3 101 special shift encoding
  STORE = 0b0100011,     // S form: SB SH SW SD
  STORE_FP = 0b0100111,  // S form: FSW FSD FSQ
  AMO = 0b0101111,       // R form: All A instructions
  OP = 0b0110011,      // R: ADD SUB SLL SLT SLTU XOR SRL SRA OR AND and 32M set
  LUI = 0b0110111,     // U form: LUI
  OP_32 = 0b0111011,   // R: ADDW SUBW SLLW SRLW SRAW MULW DIVW DIVUW REMW REMUW
  MADD = 0b1000011,    // R4 type: FMADD.S FMADD.D FMADD.Q
  MSUB = 0b1000111,    // R4 type: FMSUB.S FMSUB.D FMSUB.Q
  NMSUB = 0b1001011,   // R4 type: FNMSUB.S FNMSUB.D FNMSUB.Q
  NMADD = 0b1001111,   // R4 type: FNMADD.S FNMADD.D FNMADD.Q
  OP_FP = 0b1010011,   // R type: Q ext
  BRANCH = 0b1100011,  // B form: BEQ BNE, BLT, BGE, BLTU BGEU
  JALR = 0b1100111,    // I form: JALR
  JAL = 0b1101111,     // J form: JAL
  SYSTEM = 0b1110011,  // I form: ECALL EBREAK Zicsr ext
  OP_V = 0b1010111,    // V form: RVV

  // C extension
  C0 = 0b00,
  C1 = 0b01,
  C2 = 0b10,
  FUNCT2_0 = 0b00,
  FUNCT2_1 = 0b01,
  FUNCT2_2 = 0b10,
  FUNCT2_3 = 0b11,
};

// -----------------------------------------------------------------------------
// Specific instructions, constants, and masks.
// These constants are declared in assembler-riscv64.cc, as they use named
// registers and other constants.

// An Illegal instruction
const Instr kIllegalInstr = 0;  // All other bits are 0s (i.e., ecall)
// An ECALL instruction, used for redirected real time call
const Instr rtCallRedirInstr = SYSTEM;  // All other bits are 0s (i.e., ecall)
// An EBreak instruction, used for debugging and semi-hosting
const Instr kBreakInstr = SYSTEM | 1 << kImm12Shift;  // ebreak

constexpr uint8_t kInstrSize = 4;
constexpr uint8_t kShortInstrSize = 2;
constexpr uint8_t kInstrSizeLog2 = 2;
}
}

#endif // jit_riscv64_disasm_Constant_riscv64_h