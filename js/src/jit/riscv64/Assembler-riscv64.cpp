/* -*- Mode: C++; tab-width: 8; indent-tabs-mode: nil; c-basic-offset: 2 -*-
 * vim: set ts=8 sts=2 et sw=2 tw=80:
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */

// Copyright (c) 1994-2006 Sun Microsystems Inc.
// All Rights Reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
// - Redistributions of source code must retain the above copyright notice,
// this list of conditions and the following disclaimer.
//
// - Redistribution in binary form must reproduce the above copyright
// notice, this list of conditions and the following disclaimer in the
// documentation and/or other materials provided with the distribution.
//
// - Neither the name of Sun Microsystems or the names of contributors may
// be used to endorse or promote products derived from this software without
// specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
// IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
// THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
// PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
// CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
// EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
// PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
// LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
// NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

// The original source code covered by the above license above has been
// modified significantly by Google Inc.
// Copyright 2021 the V8 project authors. All rights reserved.
#include "jit/riscv64/Assembler-riscv64.h"

#include "mozilla/DebugOnly.h"
#include "mozilla/Maybe.h"

#include "gc/Marking.h"
#include "jit/AutoWritableJitCode.h"
#include "jit/ExecutableAllocator.h"
#include "jit/riscv64/disasm/Disasm-riscv64.h"
#include "vm/Realm.h"

using mozilla::DebugOnly;
namespace js {
namespace jit {

#define UNIMPLEMENTED_RISCV() MOZ_CRASH("RISC_V don't implement");

bool Assembler::FLAG_riscv_debug = false;

void Assembler::nop() {
  addi(ToRegister(0), ToRegister(0), 0);
}

// Size of the instruction stream, in bytes.
size_t Assembler::size() const { return m_buffer.size(); }

// Size of the relocation table, in bytes.
size_t Assembler::jumpRelocationTableBytes() const {
  return jumpRelocations_.length();
}

size_t Assembler::dataRelocationTableBytes() const {
  return dataRelocations_.length();
}
// Size of the data table, in bytes.
size_t Assembler::bytesNeeded() const {
  return size() + jumpRelocationTableBytes() + dataRelocationTableBytes();
}

void Assembler::executableCopy(uint8_t* buffer) {
  MOZ_ASSERT(isFinished);
  m_buffer.executableCopy(buffer);
}

uint32_t Assembler::AsmPoolMaxOffset = 1024;
uint32_t Assembler::NopFill = 0;

uint32_t Assembler::GetNopFill() {
  static bool isSet = false;
  if (!isSet) {
    char* fillStr = getenv("ARM_ASM_NOP_FILL");
    uint32_t fill;
    if (fillStr && sscanf(fillStr, "%u", &fill) == 1) {
      NopFill = fill;
    }
    if (NopFill > 8) {
      MOZ_CRASH("Nop fill > 8 is not supported");
    }
    isSet = true;
  }
  return NopFill;
}

uint32_t Assembler::GetPoolMaxOffset() {
  static bool isSet = false;
  if (!isSet) {
    char* poolMaxOffsetStr = getenv("ASM_POOL_MAX_OFFSET");
    uint32_t poolMaxOffset;
    if (poolMaxOffsetStr &&
        sscanf(poolMaxOffsetStr, "%u", &poolMaxOffset) == 1) {
      AsmPoolMaxOffset = poolMaxOffset;
    }
    isSet = true;
  }
  return AsmPoolMaxOffset;
}

// Pool callbacks stuff:
void Assembler::InsertIndexIntoTag(uint8_t* load_, uint32_t index) {
  MOZ_CRASH("Unimplement");
}

void Assembler::PatchConstantPoolLoad(void* loadAddr, void* constPoolAddr) {
  MOZ_CRASH("Unimplement");
}

void Assembler::processCodeLabels(uint8_t* rawCode) {
  for (const CodeLabel& label : codeLabels_) {
    Bind(rawCode, label);
  }
}

void Assembler::WritePoolGuard(BufferOffset branch, Instruction* dest,
                               BufferOffset afterPool) {
  int32_t off = afterPool.getOffset() - branch.getOffset();
  if (is_int21(off) && (off & 0x1) == 0) {
    MOZ_CRASH("imm invalid");
  }
  int32_t imm20 = (off & 0xff000) |          // bits 19-12
                  ((off & 0x800) << 9) |     // bit  11
                  ((off & 0x7fe) << 20) |    // bits 10-1
                  ((off & 0x100000) << 11);  // bit  20
  Instr instr = JAL | (imm20 & kImm20Mask);
  *reinterpret_cast<Instr*>(dest) = instr;
}

void Assembler::WritePoolHeader(uint8_t* start, Pool* p, bool isNatural) {
  MOZ_CRASH("Unimplement");
  // static_assert(sizeof(PoolHeader) == 4,
  //               "PoolHandler must have the correct size.");
  // uint8_t* pool = start + 4;
  // // Go through the usual rigmarole to get the size of the pool.
  // pool += p->getPoolSize();
  // uint32_t size = pool - start;
  // MOZ_ASSERT((size & 3) == 0);
  // size = size >> 2;
  // MOZ_ASSERT(size < (1 << 15));
  // PoolHeader header(size, isNatural);
  // *(PoolHeader*)start = header;
}


void Assembler::copyJumpRelocationTable(uint8_t* dest) {
  if (jumpRelocations_.length()) {
    memcpy(dest, jumpRelocations_.buffer(), jumpRelocations_.length());
  }
}

void Assembler::copyDataRelocationTable(uint8_t* dest) {
  if (dataRelocations_.length()) {
    memcpy(dest, dataRelocations_.buffer(), dataRelocations_.length());
  }
}

void Assembler::RV_li(Register rd, int64_t imm) {
  // 64-bit imm is put in the register rd.
  // In most cases the imm is 32 bit and 2 instructions are generated. If a
  // temporary register is available, in the worst case, 6 instructions are
  // generated for a full 64-bit immediate. If temporay register is not
  // available the maximum will be 8 instructions. If imm is more than 32 bits
  // and a temp register is available, imm is divided into two 32-bit parts,
  // low_32 and up_32. Each part is built in a separate register. low_32 is
  // built before up_32. If low_32 is negative (upper 32 bits are 1), 0xffffffff
  // is subtracted from up_32 before up_32 is built. This compensates for 32
  // bits of 1's in the lower when the two registers are added. If no temp is
  // available, the upper 32 bit is built in rd, and the lower 32 bits are
  // devided to 3 parts (11, 11, and 10 bits). The parts are shifted and added
  // to the upper part built in rd.
  if (is_int32(imm + 0x800)) {
    // 32-bit case. Maximum of 2 instructions generated
    int64_t high_20 = ((imm + 0x800) >> 12);
    int64_t low_12 = imm << 52 >> 52;
    if (high_20) {
      lui(rd, (int32_t)high_20);
      if (low_12) {
        addi(rd, rd, low_12);
      }
    } else {
      addi(rd, zero_reg, low_12);
    }
    return;
  } else {
    // 64-bit case: divide imm into two 32-bit parts, upper and lower
    int64_t up_32 = imm >> 32;
    int64_t low_32 = imm & 0xffffffffull;
    Register temp_reg = rd;
    // Check if a temporary register is available
    if (up_32 == 0 || low_32 == 0) {
      // No temp register is needed
    } else {
      UseScratchRegisterScope temps(this);
      BlockTrampolinePoolScope block_trampoline_pool(this);
      temp_reg = temps.hasAvailable() ? temps.Acquire() : InvalidReg;
    }
    if (temp_reg != InvalidReg) {
      // keep track of hardware behavior for lower part in sim_low
      int64_t sim_low = 0;
      // Build lower part
      if (low_32 != 0) {
        int64_t high_20 = ((low_32 + 0x800) >> 12);
        int64_t low_12 = low_32 & 0xfff;
        if (high_20) {
          // Adjust to 20 bits for the case of overflow
          high_20 &= 0xfffff;
          sim_low = ((high_20 << 12) << 32) >> 32;
          lui(rd, (int32_t)high_20);
          if (low_12) {
            sim_low += (low_12 << 52 >> 52) | low_12;
            addi(rd, rd, low_12);
          }
        } else {
          sim_low = low_12;
          ori(rd, zero_reg, low_12);
        }
      }
      if (sim_low & 0x100000000) {
        // Bit 31 is 1. Either an overflow or a negative 64 bit
        if (up_32 == 0) {
          // Positive number, but overflow because of the add 0x800
          slli(rd, rd, 32);
          srli(rd, rd, 32);
          return;
        }
        // low_32 is a negative 64 bit after the build
        up_32 = (up_32 - 0xffffffff) & 0xffffffff;
      }
      if (up_32 == 0) {
        return;
      }
      // Build upper part in a temporary register
      if (low_32 == 0) {
        // Build upper part in rd
        temp_reg = rd;
      }
      int64_t high_20 = (up_32 + 0x800) >> 12;
      int64_t low_12 = up_32 & 0xfff;
      if (high_20) {
        // Adjust to 20 bits for the case of overflow
        high_20 &= 0xfffff;
        lui(temp_reg, (int32_t)high_20);
        if (low_12) {
          addi(temp_reg, temp_reg, low_12);
        }
      } else {
        ori(temp_reg, zero_reg, low_12);
      }
      // Put it at the bgining of register
      slli(temp_reg, temp_reg, 32);
      if (low_32 != 0) {
        add(rd, rd, temp_reg);
      }
      return;
    }
    // No temp register. Build imm in rd.
    // Build upper 32 bits first in rd. Divide lower 32 bits parts and add
    // parts to the upper part by doing shift and add.
    // First build upper part in rd.
    int64_t high_20 = (up_32 + 0x800) >> 12;
    int64_t low_12 = up_32 & 0xfff;
    if (high_20) {
      // Adjust to 20 bits for the case of overflow
      high_20 &= 0xfffff;
      lui(rd, (int32_t)high_20);
      if (low_12) {
        addi(rd, rd, low_12);
      }
    } else {
      ori(rd, zero_reg, low_12);
    }
    // upper part already in rd. Each part to be added to rd, has maximum of 11
    // bits, and always starts with a 1. rd is shifted by the size of the part
    // plus the number of zeros between the parts. Each part is added after the
    // left shift.
    uint32_t mask = 0x80000000;
    int32_t shift_val = 0;
    int32_t i;
    for (i = 0; i < 32; i++) {
      if ((low_32 & mask) == 0) {
        mask >>= 1;
        shift_val++;
        if (i == 31) {
          // rest is zero
          slli(rd, rd, shift_val);
        }
        continue;
      }
      // The first 1 seen
      int32_t part;
      if ((i + 11) < 32) {
        // Pick 11 bits
        part = ((uint32_t)(low_32 << i) >> i) >> (32 - (i + 11));
        slli(rd, rd, shift_val + 11);
        ori(rd, rd, part);
        i += 10;
        mask >>= 11;
      } else {
        part = (uint32_t)(low_32 << i) >> i;
        slli(rd, rd, shift_val + (32 - i));
        ori(rd, rd, part);
        break;
      }
      shift_val = 0;
    }
  }
}

void Assembler::li_ptr(Register rd, int64_t imm) {
  // Initialize rd with an address
  // Pointers are 48 bits
  // 6 fixed instructions are generated
  MOZ_ASSERT((imm & 0xfff0000000000000ll) == 0);
  int64_t a6 = imm & 0x3f;                      // bits 0:5. 6 bits
  int64_t b11 = (imm >> 6) & 0x7ff;             // bits 6:11. 11 bits
  int64_t high_31 = (imm >> 17) & 0x7fffffff;   // 31 bits
  int64_t high_20 = ((high_31 + 0x800) >> 12);  // 19 bits
  int64_t low_12 = high_31 & 0xfff;             // 12 bits
  lui(rd, (int32_t)high_20);
  addi(rd, rd, low_12);  // 31 bits in rd.
  slli(rd, rd, 11);      // Space for next 11 bis
  ori(rd, rd, b11);      // 11 bits are put in. 42 bit in rd
  slli(rd, rd, 6);       // Space for next 6 bits
  ori(rd, rd, a6);       // 6 bits are put in. 48 bis in rd
}

void Assembler::li_constant(Register rd, int64_t imm) {
  DEBUG_PRINTF("li_constant(%d, %lx <%ld>)\n", ToNumber(rd), imm, imm);
  lui(rd, (imm + (1LL << 47) + (1LL << 35) + (1LL << 23) + (1LL << 11)) >>
              48);  // Bits 63:48
  addiw(rd, rd,
        (imm + (1LL << 35) + (1LL << 23) + (1LL << 11)) << 16 >>
            52);  // Bits 47:36
  slli(rd, rd, 12);
  addi(rd, rd, (imm + (1LL << 23) + (1LL << 11)) << 28 >> 52);  // Bits 35:24
  slli(rd, rd, 12);
  addi(rd, rd, (imm + (1LL << 11)) << 40 >> 52);  // Bits 23:12
  slli(rd, rd, 12);
  addi(rd, rd, imm << 52 >> 52);  // Bits 11:0
}

ABIArg ABIArgGenerator::next(MIRType type) {
  switch (type) {
    case MIRType::Int32:
    case MIRType::Int64:
    case MIRType::Pointer:
    case MIRType::RefOrNull:
    case MIRType::StackResults: {
      if (intRegIndex_ == NumIntArgRegs) {
        current_ = ABIArg(stackOffset_);
        stackOffset_ += sizeof(uintptr_t);
        break;
      }
      current_ = ABIArg(Register::FromCode(intRegIndex_ + a0.encoding()));
      intRegIndex_++;
      break;
    }
    case MIRType::Float32:
    case MIRType::Double: {
      if (floatRegIndex_ == NumFloatArgRegs) {
        current_ = ABIArg(stackOffset_);
        stackOffset_ += sizeof(double);
        break;
      }
      current_ = ABIArg(FloatRegister(
          FloatRegisters::Encoding(floatRegIndex_ + fa0.encoding()),
          type == MIRType::Double ? FloatRegisters::Double
                                  : FloatRegisters::Single));
      floatRegIndex_++;
      break;
    }
    case MIRType::Simd128: {
      MOZ_CRASH("LoongArch does not support simd yet.");
      break;
    }
    default:
      MOZ_CRASH("Unexpected argument type");
  }
  return current_;
}

bool Assembler::oom() const {
  return m_buffer.oom() || jumpRelocations_.oom() || dataRelocations_.oom();
}

void Assembler::disassembleInstr(Instr instr) {
  if (!FLAG_riscv_debug)
    return;
  disasm::NameConverter converter;
  disasm::Disassembler disasm(converter);
  EmbeddedVector<char, 128> disasm_buffer;

  disasm.InstructionDecode(disasm_buffer, reinterpret_cast<byte*>(&instr));
  DEBUG_PRINTF("%s\n", disasm_buffer.start());
}

void Assembler::BlockTrampolinePoolFor(int instructions) {
  DEBUG_PRINTF("\tBlockTrampolinePoolFor %d", instructions);
  CheckTrampolinePoolQuick(instructions);
  DEBUG_PRINTF("\tpc_offset %d,BlockTrampolinePoolBefore %d\n", currentOffset(),
               currentOffset() + instructions * kInstrSize);
  BlockTrampolinePoolBefore(currentOffset() + instructions * kInstrSize);
}

uintptr_t Assembler::target_address_at(Instruction* pc) {
  Instruction* instr0 = pc;
  DEBUG_PRINTF("target_address_at: pc: 0x%p\t", instr0);
  Instruction* instr1 = pc + 1 * kInstrSize;
  Instruction* instr2 = pc + 2 * kInstrSize;
  Instruction* instr3 = pc + 3 * kInstrSize;
  Instruction* instr4 = pc + 4 * kInstrSize;
  Instruction* instr5 = pc + 5 * kInstrSize;

  // Interpret instructions for address generated by li: See listing in
  // Assembler::set_target_address_at() just below.
  if (IsLui(*reinterpret_cast<Instr*>(instr0)) &&
      IsAddi(*reinterpret_cast<Instr*>(instr1)) &&
      IsSlli(*reinterpret_cast<Instr*>(instr2)) &&
      IsOri(*reinterpret_cast<Instr*>(instr3)) &&
      IsSlli(*reinterpret_cast<Instr*>(instr4)) &&
      IsOri(*reinterpret_cast<Instr*>(instr5))) {
    // Assemble the 64 bit value.
    int64_t addr = (int64_t)(instr0->Imm20UValue() << kImm20Shift) +
                   (int64_t)instr1->Imm12Value();
    addr <<= 11;
    addr |= (int64_t)instr3->Imm12Value();
    addr <<= 6;
    addr |= (int64_t)instr5->Imm12Value();

    DEBUG_PRINTF("addr: %lx\n", addr);
    return static_cast<uintptr_t>(addr);
  }
  // We should never get here, force a bad address if we do.
  MOZ_CRASH("RISC-V  UNREACHABLE");
}

void Assembler::set_target_value_at(Instruction* pc, uint64_t target) {
  DEBUG_PRINTF("set_target_value_at: pc: %p\ttarget: %lx\n", pc, target);
  uint32_t* p = reinterpret_cast<uint32_t*>(pc);
  MOZ_ASSERT((target & 0xffff000000000000ll) == 0);
#ifdef DEBUG
  // Check we have the result from a li macro-instruction.
  Instruction* instr0 = pc;
  Instruction* instr1 = pc + 1 * kInstrSize;
  Instruction* instr3 = pc + 3 * kInstrSize;
  Instruction* instr5 = pc + 5 * kInstrSize;
  MOZ_ASSERT(IsLui(*reinterpret_cast<Instr*>(instr0)) &&
         IsAddi(*reinterpret_cast<Instr*>(instr1)) &&
         IsOri(*reinterpret_cast<Instr*>(instr3)) &&
         IsOri(*reinterpret_cast<Instr*>(instr5)));
#endif
  int64_t a6 = target & 0x3f;                     // bits 0:6. 6 bits
  int64_t b11 = (target >> 6) & 0x7ff;            // bits 6:11. 11 bits
  int64_t high_31 = (target >> 17) & 0x7fffffff;  // 31 bits
  int64_t high_20 = ((high_31 + 0x800) >> 12);    // 19 bits
  int64_t low_12 = high_31 & 0xfff;               // 12 bits
  *p = *p & 0xfff;
  *p = *p | ((int32_t)high_20 << 12);
  *(p + 1) = *(p + 1) & 0xfffff;
  *(p + 1) = *(p + 1) | ((int32_t)low_12 << 20);
  *(p + 2) = *(p + 2) & 0xfffff;
  *(p + 2) = *(p + 2) | (11 << 20);
  *(p + 3) = *(p + 3) & 0xfffff;
  *(p + 3) = *(p + 3) | ((int32_t)b11 << 20);
  *(p + 4) = *(p + 4) & 0xfffff;
  *(p + 4) = *(p + 4) | (6 << 20);
  *(p + 5) = *(p + 5) & 0xfffff;
  *(p + 5) = *(p + 5) | ((int32_t)a6 << 20);
  MOZ_ASSERT(target_address_at(pc) == target);
}

void Assembler::target_at_put(BufferOffset pos,
                              BufferOffset target_pos,
                              bool trampoline) {
  DEBUG_PRINTF("target_at_put: %p (%d) to %p (%d)\n",
               reinterpret_cast<Instr*>(editSrc(pos)), pos.getOffset(),
               reinterpret_cast<Instr*>(editSrc(pos)) + target_pos.getOffset() -
                   pos.getOffset(),
               target_pos.getOffset());

  Instruction* instruction = editSrc(pos);
  Instr instr = instruction->InstructionBits();
  switch (instruction->InstructionOpcodeType()) {
    case BRANCH: {
      instr = SetBranchOffset(pos.getOffset(), target_pos.getOffset(), instr);
      instr_at_put(pos, instr);
    } break;
    case JAL: {
      MOZ_ASSERT(IsJal(instr));
      instr = SetJalOffset(pos.getOffset(), target_pos.getOffset(), instr);
      instr_at_put(pos, instr);
    } break;
    case LUI: {
      set_target_value_at(
          instruction, reinterpret_cast<uintptr_t>(editSrc(target_pos)));
    } break;
    case AUIPC: {
      Instr instr_auipc = instr;
      Instr instr_I = editSrc(BufferOffset(pos.getOffset() + 4))->InstructionBits();
      MOZ_ASSERT(IsJalr(instr_I) || IsAddi(instr_I));

      intptr_t offset = target_pos.getOffset() - pos.getOffset();
      if (is_int21(offset) && IsJalr(instr_I) && trampoline) {
        MOZ_ASSERT(is_int21(offset) && ((offset & 1) == 0));
        Instr instr = JAL;
        instr = SetJalOffset(pos.getOffset(), target_pos.getOffset(), instr);
        MOZ_ASSERT(IsJal(instr));
        MOZ_ASSERT(JumpOffset(instr) == offset);
        instr_at_put(pos, instr);
        instr_at_put(BufferOffset(pos.getOffset() + 4), kNopByte);
      } else {
        MOZ_RELEASE_ASSERT(is_int32(offset + 0x800));

        int32_t Hi20 = (((int32_t)offset + 0x800) >> 12);
        int32_t Lo12 = (int32_t)offset << 20 >> 20;

        instr_auipc =
            (instr_auipc & ~kImm31_12Mask) | ((Hi20 & kImm19_0Mask) << 12);
        instr_at_put(pos, instr_auipc);

        const int kImm31_20Mask = ((1 << 12) - 1) << 20;
        const int kImm11_0Mask = ((1 << 12) - 1);
        instr_I = (instr_I & ~kImm31_20Mask) | ((Lo12 & kImm11_0Mask) << 20);
        instr_at_put(BufferOffset(pos.getOffset() + 4), instr_I);
      }
    } break;
    default:
      UNIMPLEMENTED_RISCV();
      break;
  }
  disassembleInstr(instr);
}

const int kEndOfChain = -1;
const int32_t kEndOfJumpChain = 0;

int Assembler::target_at(BufferOffset pos, bool is_internal) {
  Instruction* instruction = editSrc(pos);
  DEBUG_PRINTF("target_at: %p (%d)\n\t", reinterpret_cast<Instr*>(instruction),
               pos.getOffset());
  // Instr instr = instruction->InstructionBits();
  disassembleInstr(instruction->InstructionBits());
  Instr instr = instruction->InstructionBits();
  switch (instruction->InstructionOpcodeType()) {
    case BRANCH: {
      int32_t imm13 = BranchOffset(instr);
      if (imm13 == kEndOfJumpChain) {
        // EndOfChain sentinel is returned directly, not relative to pc or pos.
        return kEndOfChain;
      } else {
        DEBUG_PRINTF("\t target_at: %d %d\n", imm13, pos.getOffset() + imm13);
        return pos.getOffset() + imm13;
      }
    }
    case JAL: {
      int32_t imm21 = JumpOffset(instr);
      if (imm21 == kEndOfJumpChain) {
        // EndOfChain sentinel is returned directly, not relative to pc or pos.
        return kEndOfChain;
      } else {
        return pos.getOffset() + imm21;
      }
    }
    case JALR: {
      int32_t imm12 = instr >> 20;
      if (imm12 == kEndOfJumpChain) {
        // EndOfChain sentinel is returned directly, not relative to pc or pos.
        return kEndOfChain;
      } else {
        return pos.getOffset() + imm12;
      }
    }
    case LUI: {
      uintptr_t imm = target_address_at(editSrc(pos));
      uintptr_t instr_address = reinterpret_cast<uintptr_t>(editSrc(pos));
      if (imm == kEndOfJumpChain) {
        return kEndOfChain;
      } else {
        MOZ_ASSERT(instr_address - imm < INT_MAX);
        int32_t delta = static_cast<int32_t>(instr_address - imm);
        MOZ_ASSERT(pos.getOffset() > delta);
        return pos.getOffset() - delta;
      }
    }
    case AUIPC: {
      Instr instr_auipc = instr;
      Instr instr_I =
          editSrc(BufferOffset(pos.getOffset() + 4))->InstructionBits();
      MOZ_ASSERT(IsJalr(instr_I) || IsAddi(instr_I));
      int32_t offset = BrachlongOffset(instr_auipc, instr_I);
      if (offset == kEndOfJumpChain)
        return kEndOfChain;
      return offset + pos.getOffset();
    }
    default: {
      UNIMPLEMENTED_RISCV();
    }
  }
}

uint32_t Assembler::next_link(Label* L, bool is_internal) {
  MOZ_ASSERT(L->used());
  BufferOffset pos(L);
  int link = target_at(pos, is_internal);
  if (link == kEndOfChain) {
    L->reset();
    return LabelBase::INVALID_OFFSET;
  } else {
    MOZ_ASSERT(link >= 0);
    DEBUG_PRINTF("next: %p to offset %d\n", L, link);
    L->use(link);
    return link;
  }
}

void Assembler::bind(Label* label, BufferOffset boff) {
  spew(".set Llabel %p", label);
  // If our caller didn't give us an explicit target to bind to
  // then we want to bind to the location of the next instruction
  BufferOffset dest = boff.assigned() ? boff : nextOffset();
  int trampoline_pos = kInvalidSlotPos;
  if (label->used()) {
    uint32_t next;

    // A used label holds a link to branch that uses it.
    do {
      BufferOffset b(label);
      DEBUG_PRINTF("\tbind next:%d\n", b.getOffset());
      // Even a 0 offset may be invalid if we're out of memory.
      if (oom()) {
        return;
      }
      int fixup_pos = b.getOffset();
      int dist = dest.getOffset() - fixup_pos;
      next = next_link(label, false);
      DEBUG_PRINTF("\t%p fixup: %d next: %d\n", label, fixup_pos, next);
      Instruction* instruction = editSrc(b);
      Instr instr = instruction->InstructionBits();
      if (IsBranch(instr)) {
        if (dist > kMaxBranchOffset) {
          if (trampoline_pos == kInvalidSlotPos) {
            trampoline_pos = get_trampoline_entry(fixup_pos);
            MOZ_RELEASE_ASSERT(trampoline_pos != kInvalidSlotPos);
          }
          MOZ_RELEASE_ASSERT((trampoline_pos - fixup_pos) <= kMaxBranchOffset);
          DEBUG_PRINTF("\t\ttrampolining: %d\n", trampoline_pos);
          target_at_put(b, BufferOffset(trampoline_pos), true);
          b = BufferOffset(trampoline_pos);
        }
        target_at_put(b, dest);
      } else if (IsJal(instr)) {
        if (dist > kMaxJumpOffset) {
          if (trampoline_pos == kInvalidSlotPos) {
            trampoline_pos = get_trampoline_entry(fixup_pos);
            MOZ_RELEASE_ASSERT(trampoline_pos != kInvalidSlotPos);
          }
          MOZ_RELEASE_ASSERT((trampoline_pos - fixup_pos) <= kMaxJumpOffset);
          DEBUG_PRINTF("\t\ttrampolining: %d\n", trampoline_pos);
          target_at_put(b, BufferOffset(trampoline_pos), true);
          b = BufferOffset(trampoline_pos);
        }
        target_at_put(b, dest);
      } else {
        target_at_put(b, dest);
      }
    } while (next != LabelBase::INVALID_OFFSET);
  }
  label->bind(dest.getOffset());
}

void Assembler::Bind(uint8_t* rawCode, const CodeLabel& label) {
  size_t offset = label.patchAt().offset();
  size_t target = label.target().offset();
  *reinterpret_cast<const void**>(rawCode + offset) = rawCode + target;
}

bool Assembler::is_near(Label* L) {
  MOZ_ASSERT(L->bound());
  return is_intn((currentOffset() - L->offset()), kJumpOffsetBits);
}

bool Assembler::is_near(Label* L, OffsetSize bits) {
  if (L == nullptr || !L->bound())
    return true;
  return is_intn((currentOffset() - L->offset()), bits);
}

bool Assembler::is_near_branch(Label* L) {
  MOZ_ASSERT(L->bound());
  return is_intn((currentOffset() - L->offset()), kBranchOffsetBits);
}

int32_t Assembler::branch_long_offset(Label* L) {
  intptr_t target_pos;

  DEBUG_PRINTF("branch_long_offset: %p to (%d)\n", L, currentOffset());
  if (L->bound()) {
    target_pos = L->offset();
  } else {
    if (L->used()) {
      target_pos = L->offset();  // L's link.
      L->use(currentOffset());
    } else {
      L->use(currentOffset());
      if (!trampoline_emitted_) {
        unbound_labels_count_++;
        next_buffer_check_ -= kTrampolineSlotsSize;
      }
      DEBUG_PRINTF("\tstarted link\n");
      return kEndOfJumpChain;
    }
  }
  intptr_t offset = target_pos - currentOffset();
  MOZ_ASSERT((offset & 3) == 0);
  MOZ_ASSERT(is_int32(offset));
  return static_cast<int32_t>(offset);
}

int32_t Assembler::branch_offset_helper(Label* L, OffsetSize bits) {
  int32_t target_pos;

  DEBUG_PRINTF("branch_offset_helper: %p to %d\n", L, currentOffset());
  if (L->bound()) {
    target_pos = L->offset();
    DEBUG_PRINTF("\tbound: %d", target_pos);
  } else {
    if (L->used()) {
      target_pos = L->offset();
      L->use(currentOffset());
      DEBUG_PRINTF("\tadded to link: %d\n", target_pos);
    } else {
      L->use(currentOffset());
      if (!trampoline_emitted_) {
        unbound_labels_count_++;
        next_buffer_check_ -= kTrampolineSlotsSize;
      }
      DEBUG_PRINTF("\tstarted link\n");
      return kEndOfJumpChain;
    }
  }

  int32_t offset = target_pos - currentOffset();
  DEBUG_PRINTF("\toffset = %d\n", offset);
  MOZ_ASSERT(is_intn(offset, bits));
  MOZ_ASSERT((offset & 1) == 0);
  return offset;
}

// Returns the next free trampoline entry.
int32_t Assembler::get_trampoline_entry(int32_t pos) {
  int32_t trampoline_entry = kInvalidSlotPos;
  if (!internal_trampoline_exception_) {
    DEBUG_PRINTF("\tstart: %d,pos: %d\n", trampoline_.start(), pos);
    if (trampoline_.start() > pos) {
      trampoline_entry = trampoline_.take_slot();
    }

    if (kInvalidSlotPos == trampoline_entry) {
      internal_trampoline_exception_ = true;
    }
  }
  return trampoline_entry;
}

void Assembler::CheckTrampolinePool() {
  // Some small sequences of instructions must not be broken up by the
  // insertion of a trampoline pool; such sequences are protected by setting
  // either trampoline_pool_blocked_nesting_ or no_trampoline_pool_before_,
  // which are both checked here. Also, recursive calls to CheckTrampolinePool
  // are blocked by trampoline_pool_blocked_nesting_.
  DEBUG_PRINTF("\tcurrentOffset %d no_trampoline_pool_before:%d\n",
               currentOffset(), no_trampoline_pool_before_);
  DEBUG_PRINTF("\ttrampoline_pool_blocked_nesting:%d\n",
               trampoline_pool_blocked_nesting_);
  if ((trampoline_pool_blocked_nesting_ > 0) ||
      (currentOffset() < no_trampoline_pool_before_)) {
    // Emission is currently blocked; make sure we try again as soon as
    // possible.
    if (trampoline_pool_blocked_nesting_ > 0) {
      next_buffer_check_ = currentOffset() + kInstrSize;
    } else {
      next_buffer_check_ = no_trampoline_pool_before_;
    }
    return;
  }

  MOZ_ASSERT(!trampoline_emitted_);
  MOZ_ASSERT(unbound_labels_count_ >= 0);
  if (unbound_labels_count_ > 0) {
    // First we emit jump, then we emit trampoline pool.
    {
      DEBUG_PRINTF("inserting trampoline pool at %d\n", currentOffset());
      BlockTrampolinePoolScope block_trampoline_pool(this);
      Label after_pool;
      j(&after_pool);

      int pool_start = currentOffset();
      for (int i = 0; i < unbound_labels_count_; i++) {
        int32_t imm;
        imm = branch_long_offset(&after_pool);
        MOZ_RELEASE_ASSERT(is_int32(imm + 0x800));
        int32_t Hi20 = (((int32_t)imm + 0x800) >> 12);
        int32_t Lo12 = (int32_t)imm << 20 >> 20;
        auipc(t6, Hi20);  // Read PC + Hi20 into t6
        jr(t6, Lo12);     // jump PC + Hi20 + Lo12
      }
      // If unbound_labels_count_ is big enough, label after_pool will
      // need a trampoline too, so we must create the trampoline before
      // the bind operation to make sure function 'bind' can get this
      // information.
      trampoline_ = Trampoline(pool_start, unbound_labels_count_);
      bind(&after_pool);

      trampoline_emitted_ = true;
      // As we are only going to emit trampoline once, we need to prevent any
      // further emission.
      next_buffer_check_ = INT32_MAX;
    }
  } else {
    // Number of branches to unbound label at this point is zero, so we can
    // move next buffer check to maximum.
    next_buffer_check_ =
        currentOffset() + kMaxBranchOffset - kTrampolineSlotsSize * 16;
  }
  return;
}

UseScratchRegisterScope::UseScratchRegisterScope(Assembler* assembler)
    : available_(assembler->GetScratchRegisterList()),
      old_available_(*available_) {}

UseScratchRegisterScope::~UseScratchRegisterScope() {
  *available_ = old_available_;
}

Register UseScratchRegisterScope::Acquire() {
  MOZ_ASSERT(available_ != nullptr);
  MOZ_ASSERT(!available_->empty());
  Register index = GeneralRegisterSet::FirstRegister(available_->bits());
  available_->takeRegisterIndex(index);
  return index;
}

bool UseScratchRegisterScope::hasAvailable() const {
  return (available_->size()) != 0;
}

}  // namespace jit
}  // namespace js
