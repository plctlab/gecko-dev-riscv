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
  return m_buffer.oom() || jumpRelocations_.oom() ||
         dataRelocations_.oom();
}

void Assembler::disassembleInstr(Instr instr) {
  if (!FLAG_riscv_debug) return;
  disasm::NameConverter converter;
  disasm::Disassembler disasm(converter);
  disasm::EmbeddedVector<char, 128> disasm_buffer;

  disasm.InstructionDecode(disasm_buffer, reinterpret_cast<byte*>(&instr));
  DEBUG_PRINTF("%s\n", disasm_buffer.start());
}

void Assembler::target_at_put(BufferOffset pos, BufferOffset target_pos) {
    DEBUG_PRINTF("target_at_put: %p (%d) to %p (%d)\n",
               reinterpret_cast<Instr*>(editSrc(pos)), pos.getOffset(),
               reinterpret_cast<Instr*>(editSrc(target_pos)),
               target_pos.getOffset());

    Instruction* instruction = editSrc(pos);
    Instr instr = instruction->InstructionBits();
    switch (instruction->InstructionOpcodeType()) {
      case BRANCH:
        UNIMPLEMENTED_RISCV();
        break;
      default:
        UNIMPLEMENTED_RISCV();
        break;
    }
    disassembleInstr(instr);
}

const int kEndOfChain = -1;
const int kEndOfJumpChain = 0;

int Assembler::target_at(BufferOffset pos, bool is_internal) {
  Instruction* instruction = editSrc(pos);
  DEBUG_PRINTF("target_at: %p (%d)\n\t", reinterpret_cast<Instr*>(instruction),
               pos.getOffset());
  Instr instr = instruction->InstructionBits();
  disassembleInstr(instruction->InstructionBits());
  switch (instruction->InstructionOpcodeType()) {
    case BRANCH: {
      UNIMPLEMENTED_RISCV();
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
    return LabelBase::INVALID_OFFSET;
  } else {
    MOZ_ASSERT(link >= 0);
    DEBUG_PRINTF("next: %p to offset %d\n", L, link);
    return link;
  }
}

void Assembler::bind(Label* label, BufferOffset boff) {
  spew(".set Llabel %p", label);
  // If our caller didn't give us an explicit target to bind to
  // then we want to bind to the location of the next instruction
  BufferOffset dest = boff.assigned() ? boff : nextOffset();
  if (label->used()) {
    int32_t next;

    // A used label holds a link to branch that uses it.
    BufferOffset b(label);
    do {
      // Even a 0 offset may be invalid if we're out of memory.
      if (oom()) {
        return;
      }
      int fixup_pos = b.getOffset();
      int dist = dest.getOffset() - fixup_pos;

      Instruction* instruction = editSrc(b);
      Instr instr = instruction->InstructionBits();
      uint32_t next = next_link(label, false);
      if (IsBranch(instr)) {
        if (dist > kMaxBranchOffset) {
          UNIMPLEMENTED_RISCV();
        }
        target_at_put(b, dest);
      }
      b = BufferOffset(next);
    } while (next != LabelBase::INVALID_OFFSET);
  }
  label->bind(dest.getOffset());
}

}  // namespace jit
}  // namespace js
