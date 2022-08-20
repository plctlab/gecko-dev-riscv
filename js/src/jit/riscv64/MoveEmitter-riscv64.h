/* -*- Mode: C++; tab-width: 8; indent-tabs-mode: nil; c-basic-offset: 2 -*-
 * vim: set ts=8 sts=2 et sw=2 tw=80:
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */

#ifndef jit_riscv64_MoveEmitter_riscv64_h
#define jit_riscv64_MoveEmitter_riscv64_h

#include "jit/shared/MoveEmitter-shared.h"

namespace js {
namespace jit {

class MoveEmitterriscv64 : public MoveEmitterRISCVShared {
  void emitDoubleMove(const MoveOperand& from, const MoveOperand& to);
  void breakCycle(const MoveOperand& from, const MoveOperand& to,
                  MoveOp::Type type, uint32_t slot);
  void completeCycle(const MoveOperand& from, const MoveOperand& to,
                     MoveOp::Type type, uint32_t slot);

 public:
  MoveEmitterriscv64(MacroAssembler& masm) : MoveEmitterRISCVShared(masm) {}
};

typedef MoveEmitterriscv64 MoveEmitter;

}  // namespace jit
}  // namespace js

#endif /* jit_riscv64_MoveEmitter_riscv64_h */
