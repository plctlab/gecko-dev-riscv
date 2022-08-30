/* -*- Mode: C++; tab-width: 8; indent-tabs-mode: nil; c-basic-offset: 2 -*-
 * vim: set ts=8 sts=2 et sw=2 tw=80:
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */

#ifndef jit_riscv64_MacroAssembler_riscv64_inl_h
#define jit_riscv64_MacroAssembler_riscv64_inl_h

#include "jit/riscv64/MacroAssembler-riscv64.h"

namespace js {
namespace jit {

template <>
inline void MacroAssembler::cmpPtrSet(Assembler::Condition cond,
                                      Address lhs,
                                      ImmPtr rhs,
                                      Register dest) {
  UseScratchRegisterScope temps(this);
  Register scratch2 = temps.Acquire();
  loadPtr(lhs, scratch2);
  cmpPtrSet(cond, Register(scratch2), rhs, dest);
}

template <>
inline void MacroAssembler::cmpPtrSet(Assembler::Condition cond,
                                      Register lhs,
                                      Address rhs,
                                      Register dest) {
  UseScratchRegisterScope temps(this);
  Register scratch = temps.Acquire();
  MOZ_ASSERT(lhs != scratch);
  loadPtr(rhs, scratch);
  cmpPtrSet(cond, lhs, Register(scratch), dest);
}

template <>
inline void MacroAssembler::cmpPtrSet(Assembler::Condition cond,
                                      Address lhs,
                                      Register rhs,
                                      Register dest) {
  UseScratchRegisterScope temps(this);
  Register scratch2 = temps.Acquire();
  MOZ_ASSERT(rhs != scratch2);
  loadPtr(lhs, scratch2);
  cmpPtrSet(cond, Register(scratch2), rhs, dest);
}

template <>
inline void MacroAssembler::cmp32Set(Assembler::Condition cond,
                                     Register lhs,
                                     Address rhs,
                                     Register dest) {
  UseScratchRegisterScope temps(this);
  Register scratch = temps.Acquire();
  MOZ_ASSERT(lhs != scratch);
  load32(rhs, scratch);
  cmp32Set(cond, lhs, Register(scratch), dest);
}

template <>
inline void MacroAssembler::cmp32Set(Assembler::Condition cond,
                                     Address lhs,
                                     Register rhs,
                                     Register dest) {
  UseScratchRegisterScope temps(this);
  Register scratch2 = temps.Acquire();
  MOZ_ASSERT(rhs != scratch2);
  load32(lhs, scratch2);
  cmp32Set(cond, Register(scratch2), rhs, dest);
}

//{{{ check_macroassembler_style
CodeOffset MacroAssembler::sub32FromStackPtrWithPatch(Register) {
  MOZ_CRASH();
}

template <class L>
void MacroAssembler::branchTest32(Condition cond,
                                  Register lhs,
                                  Imm32 rhs,
                                  L label) {
  MOZ_ASSERT(cond == Zero || cond == NonZero || cond == Signed ||
             cond == NotSigned);
  UseScratchRegisterScope temps(this);
  Register scratch = temps.Acquire();
  ma_and(scratch, lhs, rhs);
  ma_b(scratch, scratch, label, cond);
}
template <class L>
void MacroAssembler::branchTest32(Condition cond,
                                  Register lhs,
                                  Register rhs,
                                  L label) {
  MOZ_ASSERT(cond == Zero || cond == NonZero || cond == Signed ||
             cond == NotSigned);
  if (lhs == rhs) {
    ma_b(lhs, rhs, label, cond);
  } else {
    UseScratchRegisterScope temps(this);
    Register scratch = temps.Acquire();
    and_(scratch, lhs, rhs);
    ma_b(scratch, scratch, label, cond);
  }
}
template <class L>
void MacroAssembler::branchTest64(Condition cond,
                                  Register64 lhs,
                                  Register64 rhs,
                                  Register temp,
                                  L label) {
  branchTestPtr(cond, lhs.reg, rhs.reg, label);
}

template <typename T>
void MacroAssembler::branchAdd32(Condition cond,
                                 T src,
                                 Register dest,
                                 Label* overflow) {
  switch (cond) {
    case Overflow:
      ma_add32TestOverflow(dest, dest, src, overflow);
      break;
    case CarryClear:
    case CarrySet:
      ma_add32TestCarry(cond, dest, dest, src, overflow);
      break;
    default:
      MOZ_CRASH("NYI");
  }
}
template <typename T>
void MacroAssembler::branchAddPtr(Condition cond,
                                  T src,
                                  Register dest,
                                  Label* label) {
  switch (cond) {
    case Overflow:
      ma_addPtrTestOverflow(dest, dest, src, label);
      break;
    case CarryClear:
    case CarrySet:
      ma_addPtrTestCarry(cond, dest, dest, src, label);
      break;
    default:
      MOZ_CRASH("NYI");
  }
}
template <typename T>
void MacroAssembler::branchMul32(Condition cond,
                                 T src,
                                 Register dest,
                                 Label* overflow) {
  MOZ_ASSERT(cond == Assembler::Overflow);
  ma_mul32TestOverflow(dest, dest, src, overflow);
}
template <typename T>
void MacroAssembler::branchRshift32(Condition cond,
                                    T src,
                                    Register dest,
                                    Label* label) {
  MOZ_ASSERT(cond == Zero || cond == NonZero);
  rshift32(src, dest);
  branch32(cond == Zero ? Equal : NotEqual, dest, Imm32(0), label);
}
// the type of 'T src' maybe a Register, maybe a Imm32,depends on who call it.
template <typename T>
void MacroAssembler::branchSub32(Condition cond,
                                 T src,
                                 Register dest,
                                 Label* overflow) {
  switch (cond) {
    case Overflow:
      ma_sub32TestOverflow(dest, dest, src, overflow);
      break;
    case NonZero:
    case Zero:
    case Signed:
    case NotSigned:
      ma_sub32(dest, dest, src);
      ma_b(dest, dest, overflow, cond);
      break;
    default:
      MOZ_CRASH("NYI");
  }
}
template <typename T>
void MacroAssembler::branchSubPtr(Condition cond,
                                  T src,
                                  Register dest,
                                  Label* label) {
  switch (cond) {
    case Overflow:
      ma_subPtrTestOverflow(dest, dest, src, label);
      break;
    case NonZero:
    case Zero:
    case Signed:
    case NotSigned:
      subPtr(src, dest);
      ma_b(dest, dest, label, cond);
      break;
    default:
      MOZ_CRASH("NYI");
  }
}
template <typename T>
void MacroAssembler::branchTestGCThingImpl(Condition cond,
                                           const T& address,
                                           Label* label) {
  MOZ_ASSERT(cond == Equal || cond == NotEqual);
  UseScratchRegisterScope temps(this);
  Register scratch2 = temps.Acquire();
  Register tag = extractTag(address, scratch2);
  ma_b(tag, ImmTag(JS::detail::ValueLowerInclGCThingTag), label,
       (cond == Equal) ? AboveOrEqual : Below);
}
template <typename T>
void MacroAssembler::testBigIntSet(Condition, const T&, Register) {
  MOZ_CRASH();
}
template <typename T>
void MacroAssembler::testBooleanSet(Condition, const T&, Register) {
  MOZ_CRASH();
}
template <typename T>
void MacroAssembler::testNumberSet(Condition, const T&, Register) {
  MOZ_CRASH();
}
template <typename T>
void MacroAssembler::testStringSet(Condition, const T&, Register) {
  MOZ_CRASH();
}
template <typename T>
void MacroAssembler::testSymbolSet(Condition, const T&, Register) {
  MOZ_CRASH();
}
// Also see below for specializations of cmpPtrSet.
template <typename T1, typename T2>
void MacroAssembler::cmp32Set(Condition cond, T1 lhs, T2 rhs, Register dest) {
  ma_cmp_set(dest, lhs, rhs, cond);
}
template <typename T1, typename T2>
void MacroAssembler::cmpPtrSet(Condition cond, T1 lhs, T2 rhs, Register dest) {
  ma_cmp_set(dest, lhs, rhs, cond);
}
void MacroAssembler::abs32(Register src, Register dest) {
  ScratchRegisterScope scratch(asMasm());
  sraiw(scratch, src, 31);
  xor_(dest, src, scratch);
  subw(dest, dest, scratch);
}
void MacroAssembler::absFloat32(FloatRegister src, FloatRegister dest) {
  fabs_s(dest, src);
}

void MacroAssembler::absDouble(FloatRegister src, FloatRegister dest) {
  fabs_d(dest, src);
}
void MacroAssembler::add32(Register src, Register dest) {
  ma_add32(dest, dest, src);
}

void MacroAssembler::add32(Imm32 imm, Register dest) {
  ma_add32(dest, dest, imm);
}

void MacroAssembler::add32(Imm32 imm, const Address& dest) {
  UseScratchRegisterScope temps(this);
  Register scratch2 = temps.Acquire();
  load32(dest, scratch2);
  ma_add32(scratch2, scratch2, imm);
  store32(scratch2, dest);
}
void MacroAssembler::add64(Register64 src, Register64 dest) {
  addPtr(src.reg, dest.reg);
}

void MacroAssembler::add64(const Operand& src, Register64 dest) {
  if (src.is_mem()) {
    UseScratchRegisterScope temps(this);
    Register scratch = temps.Acquire();
    Register64 scratch64(scratch);

    load64(src.toAddress(), scratch64);
    add64(scratch64, dest);
  } else {
    add64(Register64(src.toReg()), dest);
  }
}

void MacroAssembler::add64(Imm32 imm, Register64 dest) {
  ma_add64(dest.reg, dest.reg, imm);
}

void MacroAssembler::add64(Imm64 imm, Register64 dest) {
  UseScratchRegisterScope temps(this);
  Register scratch = temps.Acquire();
  MOZ_ASSERT(dest.reg != scratch);
  mov(ImmWord(imm.value), scratch);
  add(dest.reg, dest.reg, scratch);
}
void MacroAssembler::addDouble(FloatRegister src, FloatRegister dest) {
  fadd_d(dest, dest, src);
}

void MacroAssembler::addFloat32(FloatRegister src, FloatRegister dest) {
  fadd_s(dest, dest, src);
}
void MacroAssembler::addPtr(Register src, Register dest) {
  ma_add64(dest, dest, Operand(src));
}

void MacroAssembler::addPtr(Imm32 imm, Register dest) {
  ma_add64(dest, dest, imm);
}

void MacroAssembler::addPtr(ImmWord imm, Register dest) {
  UseScratchRegisterScope temps(this);
  Register scratch = temps.Acquire();
  movePtr(imm, scratch);
  addPtr(scratch, dest);
}
void MacroAssembler::addPtr(Imm32 imm, const Address& dest) {
  UseScratchRegisterScope temps(this);
  Register scratch = temps.Acquire();
  loadPtr(dest, scratch);
  addPtr(imm, scratch);
  storePtr(scratch, dest);
}

void MacroAssembler::addPtr(const Address& src, Register dest) {
  UseScratchRegisterScope temps(this);
  Register scratch = temps.Acquire();
  loadPtr(src, scratch);
  addPtr(scratch, dest);
}
void MacroAssembler::and32(Register src, Register dest) {
  ma_and(dest, dest, src);
}

void MacroAssembler::and32(Imm32 imm, Register dest) {
  ma_and(dest, dest, imm);
}

void MacroAssembler::and32(Imm32 imm, const Address& dest) {
  UseScratchRegisterScope temps(this);
  Register scratch2 = temps.Acquire();
  load32(dest, scratch2);
  ma_and(scratch2, imm);
  store32(scratch2, dest);
}

void MacroAssembler::and32(const Address& src, Register dest) {
  UseScratchRegisterScope temps(this);
  Register scratch2 = temps.Acquire();
  load32(src, scratch2);
  ma_and(dest, dest, scratch2);
}
void MacroAssembler::and64(Imm64 imm, Register64 dest) {
  UseScratchRegisterScope temps(this);
  Register scratch = temps.Acquire();
  ma_li(scratch, ImmWord(imm.value));
  ma_and(dest.reg, dest.reg, scratch);
}

void MacroAssembler::and64(Register64 src, Register64 dest) {
  ma_and(dest.reg, dest.reg, src.reg);
}

void MacroAssembler::and64(const Operand& src, Register64 dest) {
  if (src.is_mem()) {
    UseScratchRegisterScope temps(this);
    Register scratch = temps.Acquire();
    Register64 scratch64(scratch);

    load64(src.toAddress(), scratch64);
    ma_and(dest.scratchReg(), scratch64.scratchReg());
  } else {
    ma_and(dest.scratchReg(), src.toReg());
  }
}

void MacroAssembler::andPtr(Register src, Register dest) {
  ma_and(dest, dest, src);
}

void MacroAssembler::andPtr(Imm32 imm, Register dest) {
  ma_and(dest, dest, imm);
}
void MacroAssembler::branch16(Condition cond,
                              const Address& lhs,
                              Imm32 rhs,
                              Label* label) {
  UseScratchRegisterScope temps(this);
  Register scratch2 = temps.Acquire();
  MOZ_ASSERT(scratch2 != lhs.base);

  switch (cond) {
    case Assembler::Equal:
    case Assembler::NotEqual:
    case Assembler::Above:
    case Assembler::AboveOrEqual:
    case Assembler::Below:
    case Assembler::BelowOrEqual:
      load16ZeroExtend(lhs, scratch2);
      branch32(cond, scratch2, Imm32(uint16_t(rhs.value)), label);
      break;

    case Assembler::GreaterThan:
    case Assembler::GreaterThanOrEqual:
    case Assembler::LessThan:
    case Assembler::LessThanOrEqual:
      load16SignExtend(lhs, scratch2);
      branch32(cond, scratch2, Imm32(int16_t(rhs.value)), label);
      break;

    default:
      MOZ_CRASH("unexpected condition");
  }
}
template <class L>
void MacroAssembler::branch32(Condition cond,
                              Register lhs,
                              Register rhs,
                              L label) {
  ma_b(lhs, rhs, label, cond);
}

template <class L>
void MacroAssembler::branch32(Condition cond,
                              Register lhs,
                              Imm32 imm,
                              L label) {
  ma_b(lhs, imm, label, cond);
}

void MacroAssembler::branch32(Condition cond,
                              const Address& lhs,
                              Register rhs,
                              Label* label) {
  UseScratchRegisterScope temps(this);
  Register scratch2 = temps.Acquire();
  load32(lhs, scratch2);
  ma_b(scratch2, rhs, label, cond);
}

void MacroAssembler::branch32(Condition cond,
                              const Address& lhs,
                              Imm32 rhs,
                              Label* label) {
  UseScratchRegisterScope temps(this);
  Register scratch2 = temps.Acquire();
  load32(lhs, scratch2);
  ma_b(scratch2, rhs, label, cond);
}

void MacroAssembler::branch32(Condition cond,
                              const AbsoluteAddress& lhs,
                              Register rhs,
                              Label* label) {
  UseScratchRegisterScope temps(this);
  Register scratch2 = temps.Acquire();
  load32(lhs, scratch2);
  ma_b(scratch2, rhs, label, cond);
}

void MacroAssembler::branch32(Condition cond,
                              const AbsoluteAddress& lhs,
                              Imm32 rhs,
                              Label* label) {
  UseScratchRegisterScope temps(this);
  Register scratch2 = temps.Acquire();
  load32(lhs, scratch2);
  ma_b(scratch2, rhs, label, cond);
}

void MacroAssembler::branch32(Condition cond,
                              const BaseIndex& lhs,
                              Imm32 rhs,
                              Label* label) {
  UseScratchRegisterScope temps(this);
  Register scratch2 = temps.Acquire();
  load32(lhs, scratch2);
  ma_b(scratch2, rhs, label, cond);
}

void MacroAssembler::branch32(Condition cond,
                              wasm::SymbolicAddress addr,
                              Imm32 imm,
                              Label* label) {
  UseScratchRegisterScope temps(this);
  Register scratch2 = temps.Acquire();
  load32(addr, scratch2);
  ma_b(scratch2, imm, label, cond);
}
void MacroAssembler::branch64(Condition cond,
                              Register64 lhs,
                              Imm64 val,
                              Label* success,
                              Label* fail) {
  MOZ_ASSERT(cond == Assembler::NotEqual || cond == Assembler::Equal ||
                 cond == Assembler::LessThan ||
                 cond == Assembler::LessThanOrEqual ||
                 cond == Assembler::GreaterThan ||
                 cond == Assembler::GreaterThanOrEqual ||
                 cond == Assembler::Below || cond == Assembler::BelowOrEqual ||
                 cond == Assembler::Above || cond == Assembler::AboveOrEqual,
             "other condition codes not supported");

  branchPtr(cond, lhs.reg, ImmWord(val.value), success);
  if (fail) {
    jump(fail);
  }
}

void MacroAssembler::branch64(Condition cond,
                              Register64 lhs,
                              Register64 rhs,
                              Label* success,
                              Label* fail) {
  MOZ_ASSERT(cond == Assembler::NotEqual || cond == Assembler::Equal ||
                 cond == Assembler::LessThan ||
                 cond == Assembler::LessThanOrEqual ||
                 cond == Assembler::GreaterThan ||
                 cond == Assembler::GreaterThanOrEqual ||
                 cond == Assembler::Below || cond == Assembler::BelowOrEqual ||
                 cond == Assembler::Above || cond == Assembler::AboveOrEqual,
             "other condition codes not supported");

  branchPtr(cond, lhs.reg, rhs.reg, success);
  if (fail) {
    jump(fail);
  }
}

void MacroAssembler::branch64(Condition cond,
                              const Address& lhs,
                              Imm64 val,
                              Label* label) {
  MOZ_ASSERT(cond == Assembler::NotEqual || cond == Assembler::Equal,
             "other condition codes not supported");

  branchPtr(cond, lhs, ImmWord(val.value), label);
}

void MacroAssembler::branch64(Condition cond,
                              const Address& lhs,
                              Register64 rhs,
                              Label* label) {
  MOZ_ASSERT(cond == Assembler::NotEqual || cond == Assembler::Equal,
             "other condition codes not supported");

  branchPtr(cond, lhs, rhs.reg, label);
}

void MacroAssembler::branch64(Condition cond,
                              const Address& lhs,
                              const Address& rhs,
                              Register scratch,
                              Label* label) {
  MOZ_ASSERT(cond == Assembler::NotEqual || cond == Assembler::Equal,
             "other condition codes not supported");
  MOZ_ASSERT(lhs.base != scratch);
  MOZ_ASSERT(rhs.base != scratch);

  loadPtr(rhs, scratch);
  branchPtr(cond, lhs, scratch, label);
}

void MacroAssembler::branch8(Condition cond,
                             const Address& lhs,
                             Imm32 rhs,
                             Label* label) {
  UseScratchRegisterScope temps(this);
  Register scratch2 = temps.Acquire();
  MOZ_ASSERT(scratch2 != lhs.base);

  switch (cond) {
    case Assembler::Equal:
    case Assembler::NotEqual:
    case Assembler::Above:
    case Assembler::AboveOrEqual:
    case Assembler::Below:
    case Assembler::BelowOrEqual:
      load8ZeroExtend(lhs, scratch2);
      branch32(cond, scratch2, Imm32(uint8_t(rhs.value)), label);
      break;

    case Assembler::GreaterThan:
    case Assembler::GreaterThanOrEqual:
    case Assembler::LessThan:
    case Assembler::LessThanOrEqual:
      load8SignExtend(lhs, scratch2);
      branch32(cond, scratch2, Imm32(int8_t(rhs.value)), label);
      break;

    default:
      MOZ_CRASH("unexpected condition");
  }
}

void MacroAssembler::branchDouble(DoubleCondition cc,
                                  FloatRegister frs1,
                                  FloatRegister frs2,
                                  Label* L) {
  UseScratchRegisterScope temps(this);
  Register scratch = temps.Acquire();
  ma_compareF64(scratch, cc, frs1, frs2);
  ma_b(scratch, Imm32(1), L, Equal);
}
void MacroAssembler::branchFloat(DoubleCondition cc,
                                 FloatRegister frs1,
                                 FloatRegister frs2,
                                 Label* L) {
  UseScratchRegisterScope temps(this);
  Register scratch = temps.Acquire();
  ma_compareF32(scratch, cc, frs1, frs2);
  ma_b(scratch, Imm32(1), L, Equal);
}
void MacroAssembler::branchMulPtr(Condition cond,
                                  Register src,
                                  Register dest,
                                  Label* label) {
  MOZ_ASSERT(cond == Assembler::Overflow);
  ma_mulPtrTestOverflow(dest, dest, src, label);
}
void MacroAssembler::branchNeg32(Condition cond, Register reg, Label* label) {
  MOZ_ASSERT(cond == Overflow);
  neg32(reg);
  branch32(Assembler::Equal, reg, Imm32(INT32_MIN), label);
}

void MacroAssembler::branchPrivatePtr(Condition cond,
                                      const Address& lhs,
                                      Register rhs,
                                      Label* label) {
  branchPtr(cond, lhs, rhs, label);
}

template <class L>
void MacroAssembler::branchPtr(Condition cond,
                               Register lhs,
                               Register rhs,
                               L label) {
  ma_b(lhs, rhs, label, cond);
}

void MacroAssembler::branchPtr(Condition cond,
                               Register lhs,
                               Imm32 rhs,
                               Label* label) {
  ma_b(lhs, rhs, label, cond);
}

void MacroAssembler::branchPtr(Condition cond,
                               Register lhs,
                               ImmPtr rhs,
                               Label* label) {
  ma_b(lhs, rhs, label, cond);
}

void MacroAssembler::branchPtr(Condition cond,
                               Register lhs,
                               ImmGCPtr rhs,
                               Label* label) {
  ma_b(lhs, rhs, label, cond);
}

void MacroAssembler::branchPtr(Condition cond,
                               Register lhs,
                               ImmWord rhs,
                               Label* label) {
  ma_b(lhs, rhs, label, cond);
}

template <class L>
void MacroAssembler::branchPtr(Condition cond,
                               const Address& lhs,
                               Register rhs,
                               L label) {
  UseScratchRegisterScope temps(this);
  Register scratch2 = temps.Acquire();
  loadPtr(lhs, scratch2);
  branchPtr(cond, scratch2, rhs, label);
}

void MacroAssembler::branchPtr(Condition cond,
                               const Address& lhs,
                               ImmPtr rhs,
                               Label* label) {
  UseScratchRegisterScope temps(this);
  Register scratch2 = temps.Acquire();
  loadPtr(lhs, scratch2);
  branchPtr(cond, scratch2, rhs, label);
}

void MacroAssembler::branchPtr(Condition cond,
                               const Address& lhs,
                               ImmGCPtr rhs,
                               Label* label) {
  UseScratchRegisterScope temps(this);
  Register scratch2 = temps.Acquire();
  loadPtr(lhs, scratch2);
  branchPtr(cond, scratch2, rhs, label);
}

void MacroAssembler::branchPtr(Condition cond,
                               const Address& lhs,
                               ImmWord rhs,
                               Label* label) {
  UseScratchRegisterScope temps(this);
  Register scratch2 = temps.Acquire();
  loadPtr(lhs, scratch2);
  branchPtr(cond, scratch2, rhs, label);
}

void MacroAssembler::branchPtr(Condition cond,
                               const AbsoluteAddress& lhs,
                               Register rhs,
                               Label* label) {
  UseScratchRegisterScope temps(this);
  Register scratch2 = temps.Acquire();
  loadPtr(lhs, scratch2);
  branchPtr(cond, scratch2, rhs, label);
}

void MacroAssembler::branchPtr(Condition cond,
                               const AbsoluteAddress& lhs,
                               ImmWord rhs,
                               Label* label) {
  UseScratchRegisterScope temps(this);
  Register scratch2 = temps.Acquire();
  loadPtr(lhs, scratch2);
  branchPtr(cond, scratch2, rhs, label);
}

void MacroAssembler::branchPtr(Condition cond,
                               wasm::SymbolicAddress lhs,
                               Register rhs,
                               Label* label) {
  UseScratchRegisterScope temps(this);
  Register scratch2 = temps.Acquire();
  loadPtr(lhs, scratch2);
  branchPtr(cond, scratch2, rhs, label);
}

void MacroAssembler::branchPtr(Condition cond,
                               const BaseIndex& lhs,
                               Register rhs,
                               Label* label) {
  UseScratchRegisterScope temps(this);
  Register scratch2 = temps.Acquire();
  loadPtr(lhs, scratch2);
  branchPtr(cond, scratch2, rhs, label);
}

void MacroAssembler::branchPtr(Condition cond,
                               const BaseIndex& lhs,
                               ImmWord rhs,
                               Label* label) {
  UseScratchRegisterScope temps(this);
  Register scratch2 = temps.Acquire();
  loadPtr(lhs, scratch2);
  branchPtr(cond, scratch2, rhs, label);
}

void MacroAssembler::branchTest32(Condition cond,
                                  const AbsoluteAddress& lhs,
                                  Imm32 rhs,
                                  Label* label) {
  MOZ_ASSERT(cond == Zero || cond == NonZero || cond == Signed ||
             cond == NotSigned);
  UseScratchRegisterScope temps(this);
  Register scratch2 = temps.Acquire();
  load32(lhs, scratch2);
  and32(rhs, scratch2);
  ma_b(scratch2, scratch2, label, cond);
}

void MacroAssembler::branchTest32(Condition cond,
                                  const Address& lhs,
                                  Imm32 rhs,
                                  Label* label) {
  MOZ_ASSERT(cond == Zero || cond == NonZero || cond == Signed ||
             cond == NotSigned);
  UseScratchRegisterScope temps(this);
  Register scratch2 = temps.Acquire();
  load32(lhs, scratch2);
  and32(rhs, scratch2);
  ma_b(scratch2, scratch2, label, cond);
}
void MacroAssembler::branchTestBigInt(Condition cond,
                                      Register tag,
                                      Label* label) {
  MOZ_ASSERT(cond == Equal || cond == NotEqual);
  ma_b(tag, ImmTag(JSVAL_TAG_BIGINT), label, cond);
}

void MacroAssembler::branchTestBigInt(Condition cond,
                                      const ValueOperand& value,
                                      Label* label) {
  UseScratchRegisterScope temps(this);
  Register scratch2 = temps.Acquire();
  splitTag(value, scratch2);
  branchTestBigInt(cond, scratch2, label);
}

void MacroAssembler::branchTestBigInt(Condition cond,
                                      const Address& address,
                                      Label* label) {
  UseScratchRegisterScope temps(this);
  Register scratch2 = temps.Acquire();
  Register tag = extractTag(address, scratch2);
  branchTestBigInt(cond, tag, label);
}

void MacroAssembler::branchTestBigInt(Condition cond,
                                      const BaseIndex& address,
                                      Label* label) {
  UseScratchRegisterScope temps(this);
  Register scratch2 = temps.Acquire();
  computeEffectiveAddress(address, scratch2);
  splitTag(scratch2, scratch2);
  branchTestBigInt(cond, scratch2, label);
}
void MacroAssembler::branchTestBigIntTruthy(bool b,
                                            const ValueOperand& value,
                                            Label* label) {
  UseScratchRegisterScope temps(this);
  Register scratch2 = temps.Acquire();
  unboxBigInt(value, scratch2);
  load32(Address(scratch2, BigInt::offsetOfDigitLength()), scratch2);
  ma_b(scratch2, Imm32(0), label, b ? NotEqual : Equal);
}
void MacroAssembler::branchTestBoolean(Condition cond,
                                       Register tag,
                                       Label* label) {
  MOZ_ASSERT(cond == Equal || cond == NotEqual);
  ma_b(tag, ImmTag(JSVAL_TAG_BOOLEAN), label, cond);
}

void MacroAssembler::branchTestBoolean(Condition cond,
                                       const ValueOperand& value,
                                       Label* label) {
  UseScratchRegisterScope temps(this);
  Register scratch2 = temps.Acquire();
  splitTag(value, scratch2);
  branchTestBoolean(cond, scratch2, label);
}

void MacroAssembler::branchTestBoolean(Condition cond,
                                       const Address& address,
                                       Label* label) {
  UseScratchRegisterScope temps(this);
  Register scratch2 = temps.Acquire();
  Register tag = extractTag(address, scratch2);
  branchTestBoolean(cond, tag, label);
}

void MacroAssembler::branchTestBoolean(Condition cond,
                                       const BaseIndex& address,
                                       Label* label) {
  UseScratchRegisterScope temps(this);
  Register scratch2 = temps.Acquire();
  Register tag = extractTag(address, scratch2);
  branchTestBoolean(cond, tag, label);
}
void MacroAssembler::branchTestBooleanTruthy(bool b,
                                             const ValueOperand& value,
                                             Label* label) {
  UseScratchRegisterScope temps(this);
  Register scratch2 = temps.Acquire();
  unboxBoolean(value, scratch2);
  ma_b(scratch2, scratch2, label, b ? NonZero : Zero);
}
void MacroAssembler::branchTestDouble(Condition cond,
                                      Register tag,
                                      Label* label) {
  MOZ_ASSERT(cond == Equal || cond == NotEqual);
  Condition actual = (cond == Equal) ? BelowOrEqual : Above;
  ma_b(tag, ImmTag(JSVAL_TAG_MAX_DOUBLE), label, actual);
}

void MacroAssembler::branchTestDouble(Condition cond,
                                      const ValueOperand& value,
                                      Label* label) {
  UseScratchRegisterScope temps(this);
  Register scratch2 = temps.Acquire();
  splitTag(value, scratch2);
  branchTestDouble(cond, scratch2, label);
}

void MacroAssembler::branchTestDouble(Condition cond,
                                      const Address& address,
                                      Label* label) {
  UseScratchRegisterScope temps(this);
  Register scratch2 = temps.Acquire();
  Register tag = extractTag(address, scratch2);
  branchTestDouble(cond, tag, label);
}

void MacroAssembler::branchTestDouble(Condition cond,
                                      const BaseIndex& address,
                                      Label* label) {
  UseScratchRegisterScope temps(this);
  Register scratch2 = temps.Acquire();
  Register tag = extractTag(address, scratch2);
  branchTestDouble(cond, tag, label);
}

void MacroAssembler::branchTestDoubleTruthy(bool, FloatRegister, Label*) {
  MOZ_CRASH();
}
void MacroAssembler::branchTestGCThing(Condition cond,
                                       const Address& address,
                                       Label* label) {
  branchTestGCThingImpl(cond, address, label);
}

void MacroAssembler::branchTestGCThing(Condition cond,
                                       const BaseIndex& address,
                                       Label* label) {
  branchTestGCThingImpl(cond, address, label);
}

void MacroAssembler::branchTestGCThing(Condition cond,
                                       const ValueOperand& address,
                                       Label* label) {
  branchTestGCThingImpl(cond, address, label);
}
void MacroAssembler::branchTestInt32(Condition cond,
                                     Register tag,
                                     Label* label) {
  MOZ_ASSERT(cond == Equal || cond == NotEqual);
  ma_b(tag, ImmTag(JSVAL_TAG_INT32), label, cond);
}

void MacroAssembler::branchTestInt32(Condition cond,
                                     const ValueOperand& value,
                                     Label* label) {
  UseScratchRegisterScope temps(this);
  Register scratch2 = temps.Acquire();
  splitTag(value, scratch2);
  branchTestInt32(cond, scratch2, label);
}

void MacroAssembler::branchTestInt32(Condition cond,
                                     const Address& address,
                                     Label* label) {
  UseScratchRegisterScope temps(this);
  Register scratch2 = temps.Acquire();
  Register tag = extractTag(address, scratch2);
  branchTestInt32(cond, tag, label);
}

void MacroAssembler::branchTestInt32(Condition cond,
                                     const BaseIndex& address,
                                     Label* label) {
  UseScratchRegisterScope temps(this);
  Register scratch2 = temps.Acquire();
  Register tag = extractTag(address, scratch2);
  branchTestInt32(cond, tag, label);
}
void MacroAssembler::branchTestInt32Truthy(bool, const ValueOperand&, Label*) {
  MOZ_CRASH();
}
void MacroAssembler::branchTestMagic(Condition cond,
                                     Register tag,
                                     Label* label) {
  MOZ_ASSERT(cond == Equal || cond == NotEqual);
  ma_b(tag, ImmTag(JSVAL_TAG_MAGIC), label, cond);
}

void MacroAssembler::branchTestMagic(Condition cond,
                                     const Address& address,
                                     Label* label) {
  UseScratchRegisterScope temps(this);
  Register scratch2 = temps.Acquire();
  Register tag = extractTag(address, scratch2);
  branchTestMagic(cond, tag, label);
}

void MacroAssembler::branchTestMagic(Condition cond,
                                     const BaseIndex& address,
                                     Label* label) {
  UseScratchRegisterScope temps(this);
  Register scratch2 = temps.Acquire();
  Register tag = extractTag(address, scratch2);
  branchTestMagic(cond, tag, label);
}

template <class L>
void MacroAssembler::branchTestMagic(Condition cond,
                                     const ValueOperand& value,
                                     L label) {
  UseScratchRegisterScope temps(this);
  Register scratch2 = temps.Acquire();
  splitTag(value, scratch2);
  ma_b(scratch2, ImmTag(JSVAL_TAG_MAGIC), label, cond);
}

void MacroAssembler::branchTestMagic(Condition cond,
                                     const Address& valaddr,
                                     JSWhyMagic why,
                                     Label* label) {
  uint64_t magic = MagicValue(why).asRawBits();
  UseScratchRegisterScope temps(this);
  Register scratch = temps.Acquire();
  loadPtr(valaddr, scratch);
  ma_b(scratch, ImmWord(magic), label, cond);
}
void MacroAssembler::branchTestNull(Condition cond,
                                    Register tag,
                                    Label* label) {
  MOZ_ASSERT(cond == Equal || cond == NotEqual);
  ma_b(tag, ImmTag(JSVAL_TAG_NULL), label, cond);
}

void MacroAssembler::branchTestNull(Condition cond,
                                    const ValueOperand& value,
                                    Label* label) {
  UseScratchRegisterScope temps(this);
  Register scratch2 = temps.Acquire();
  splitTag(value, scratch2);
  branchTestNull(cond, scratch2, label);
}

void MacroAssembler::branchTestNull(Condition cond,
                                    const Address& address,
                                    Label* label) {
  UseScratchRegisterScope temps(this);
  Register scratch2 = temps.Acquire();
  Register tag = extractTag(address, scratch2);
  branchTestNull(cond, tag, label);
}

void MacroAssembler::branchTestNull(Condition cond,
                                    const BaseIndex& address,
                                    Label* label) {
  UseScratchRegisterScope temps(this);
  Register scratch2 = temps.Acquire();
  Register tag = extractTag(address, scratch2);
  branchTestNull(cond, tag, label);
}
void MacroAssembler::branchTestNumber(Condition cond,
                                      Register tag,
                                      Label* label) {
  MOZ_ASSERT(cond == Equal || cond == NotEqual);
  Condition actual = cond == Equal ? BelowOrEqual : Above;
  ma_b(tag, ImmTag(JS::detail::ValueUpperInclNumberTag), label, actual);
}

void MacroAssembler::branchTestNumber(Condition cond,
                                      const ValueOperand& value,
                                      Label* label) {
  UseScratchRegisterScope temps(this);
  Register scratch2 = temps.Acquire();
  splitTag(value, scratch2);
  branchTestNumber(cond, scratch2, label);
}
void MacroAssembler::branchTestObject(Condition cond,
                                      Register tag,
                                      Label* label) {
  MOZ_ASSERT(cond == Equal || cond == NotEqual);
  ma_b(tag, ImmTag(JSVAL_TAG_OBJECT), label, cond);
}

void MacroAssembler::branchTestObject(Condition cond,
                                      const ValueOperand& value,
                                      Label* label) {
  UseScratchRegisterScope temps(this);
  Register scratch2 = temps.Acquire();
  splitTag(value, scratch2);
  branchTestObject(cond, scratch2, label);
}

void MacroAssembler::branchTestObject(Condition cond,
                                      const Address& address,
                                      Label* label) {
  UseScratchRegisterScope temps(this);
  Register scratch2 = temps.Acquire();
  Register tag = extractTag(address, scratch2);
  branchTestObject(cond, tag, label);
}

void MacroAssembler::branchTestObject(Condition cond,
                                      const BaseIndex& address,
                                      Label* label) {
  UseScratchRegisterScope temps(this);
  Register scratch2 = temps.Acquire();
  Register tag = extractTag(address, scratch2);
  branchTestObject(cond, tag, label);
}
void MacroAssembler::branchTestPrimitive(Condition,
                                         const ValueOperand&,
                                         Label*) {
  MOZ_CRASH();
}
void MacroAssembler::branchTestPrimitive(Condition, Register, Label*) {
  MOZ_CRASH();
}
template <class L>
void MacroAssembler::branchTestPtr(Condition cond,
                                   Register lhs,
                                   Register rhs,
                                   L label) {
  MOZ_ASSERT(cond == Zero || cond == NonZero || cond == Signed ||
             cond == NotSigned);
  if (lhs == rhs) {
    ma_b(lhs, rhs, label, cond);
  } else {
    UseScratchRegisterScope temps(this);
    Register scratch = temps.Acquire();
    ma_and(scratch, lhs, Operand(rhs));
    ma_b(scratch, scratch, label, cond);
  }
}

void MacroAssembler::branchTestPtr(Condition cond,
                                   Register lhs,
                                   Imm32 rhs,
                                   Label* label) {
  MOZ_ASSERT(cond == Zero || cond == NonZero || cond == Signed ||
             cond == NotSigned);
  UseScratchRegisterScope temps(this);
  Register scratch = temps.Acquire();
  ma_and(scratch, lhs, rhs);
  ma_b(scratch, scratch, label, cond);
}

void MacroAssembler::branchTestPtr(Condition cond,
                                   const Address& lhs,
                                   Imm32 rhs,
                                   Label* label) {
  UseScratchRegisterScope temps(this);
  Register scratch2 = temps.Acquire();
  loadPtr(lhs, scratch2);
  branchTestPtr(cond, scratch2, rhs, label);
}
void MacroAssembler::branchTestString(Condition cond,
                                      Register tag,
                                      Label* label) {
  MOZ_ASSERT(cond == Equal || cond == NotEqual);
  ma_b(tag, ImmTag(JSVAL_TAG_STRING), label, cond);
}

void MacroAssembler::branchTestString(Condition cond,
                                      const ValueOperand& value,
                                      Label* label) {
  UseScratchRegisterScope temps(this);
  Register scratch2 = temps.Acquire();
  splitTag(value, scratch2);
  branchTestString(cond, scratch2, label);
}

void MacroAssembler::branchTestString(Condition cond,
                                      const Address& address,
                                      Label* label) {
  UseScratchRegisterScope temps(this);
  Register scratch2 = temps.Acquire();
  Register tag = extractTag(address, scratch2);
  branchTestString(cond, tag, label);
}

void MacroAssembler::branchTestString(Condition cond,
                                      const BaseIndex& address,
                                      Label* label) {
  UseScratchRegisterScope temps(this);
  Register scratch2 = temps.Acquire();
  Register tag = extractTag(address, scratch2);
  branchTestString(cond, tag, label);
}
void MacroAssembler::branchTestStringTruthy(bool b,
                                            const ValueOperand& value,
                                            Label* label) {
  UseScratchRegisterScope temps(this);
  Register scratch2 = temps.Acquire();
  unboxString(value, scratch2);
  load32(Address(scratch2, JSString::offsetOfLength()), scratch2);
  ma_b(scratch2, Imm32(0), label, b ? NotEqual : Equal);
}
void MacroAssembler::branchTestSymbol(Condition cond,
                                      Register tag,
                                      Label* label) {
  MOZ_ASSERT(cond == Equal || cond == NotEqual);
  ma_b(tag, ImmTag(JSVAL_TAG_SYMBOL), label, cond);
}

void MacroAssembler::branchTestSymbol(Condition cond,
                                      const ValueOperand& value,
                                      Label* label) {
  UseScratchRegisterScope temps(this);
  Register scratch2 = temps.Acquire();
  splitTag(value, scratch2);
  branchTestSymbol(cond, scratch2, label);
}

void MacroAssembler::branchTestSymbol(Condition cond,
                                      const BaseIndex& address,
                                      Label* label) {
  UseScratchRegisterScope temps(this);
  Register scratch2 = temps.Acquire();
  Register tag = extractTag(address, scratch2);
  branchTestSymbol(cond, tag, label);
}

void MacroAssembler::branchTestSymbol(Condition cond,
                                      const Address& address,
                                      Label* label) {
  UseScratchRegisterScope temps(this);
  Register scratch2 = temps.Acquire();
  Register tag = extractTag(address, scratch2);
  branchTestSymbol(cond, tag, label);
}
void MacroAssembler::branchTestUndefined(Condition cond,
                                         const ValueOperand& value,
                                         Label* label) {
  UseScratchRegisterScope temps(this);
  Register scratch2 = temps.Acquire();
  splitTag(value, scratch2);
  branchTestUndefined(cond, scratch2, label);
}

void MacroAssembler::branchTestUndefined(Condition cond,
                                         const Address& address,
                                         Label* label) {
  UseScratchRegisterScope temps(this);
  Register scratch2 = temps.Acquire();
  Register tag = extractTag(address, scratch2);
  branchTestUndefined(cond, tag, label);
}

void MacroAssembler::branchTestUndefined(Condition cond,
                                         const BaseIndex& address,
                                         Label* label) {
  UseScratchRegisterScope temps(this);
  Register scratch2 = temps.Acquire();
  Register tag = extractTag(address, scratch2);
  branchTestUndefined(cond, tag, label);
}
void MacroAssembler::branchTestUndefined(Condition cond,
                                         Register tag,
                                         Label* label) {
  MOZ_ASSERT(cond == Equal || cond == NotEqual);
  ma_b(tag, ImmTag(JSVAL_TAG_UNDEFINED), label, cond);
}
void MacroAssembler::branchTestValue(Condition cond,
                                     const BaseIndex& lhs,
                                     const ValueOperand& rhs,
                                     Label* label) {
  MOZ_ASSERT(cond == Assembler::Equal || cond == Assembler::NotEqual);
  branchPtr(cond, lhs, rhs.valueReg(), label);
}
void MacroAssembler::branchToComputedAddress(const BaseIndex& addr) {
  UseScratchRegisterScope temps(this);
  Register scratch2 = temps.Acquire();
  loadPtr(addr, scratch2);
  branch(scratch2);
}
void MacroAssembler::branchTruncateDoubleMaybeModUint32(FloatRegister,
                                                        Register,
                                                        Label*) {
  MOZ_CRASH();
}
void MacroAssembler::branchTruncateDoubleToInt32(FloatRegister,
                                                 Register,
                                                 Label*) {
  MOZ_CRASH();
}
void MacroAssembler::branchTruncateFloat32MaybeModUint32(FloatRegister,
                                                         Register,
                                                         Label*) {
  MOZ_CRASH();
}
void MacroAssembler::branchTruncateFloat32ToInt32(FloatRegister,
                                                  Register,
                                                  Label*) {
  MOZ_CRASH();
}
void MacroAssembler::byteSwap16SignExtend(Register) {
  MOZ_CRASH();
}
void MacroAssembler::byteSwap16ZeroExtend(Register) {
  MOZ_CRASH();
}
void MacroAssembler::byteSwap32(Register) {
  MOZ_CRASH();
}
void MacroAssembler::byteSwap64(Register64) {
  MOZ_CRASH();
}
void MacroAssembler::clampIntToUint8(Register) {
  MOZ_CRASH();
}
void MacroAssembler::clz32(Register, Register, bool) {
  MOZ_CRASH();
}
void MacroAssembler::clz64(Register64, Register) {
  MOZ_CRASH();
}
void MacroAssembler::cmp16Set(Condition, Address, Imm32, Register) {
  MOZ_CRASH();
}
void MacroAssembler::cmp32Load32(Condition,
                                 Register,
                                 const Address&,
                                 const Address&,
                                 Register) {
  MOZ_CRASH();
}
void MacroAssembler::cmp32Load32(Condition,
                                 Register,
                                 Register,
                                 const Address&,
                                 Register) {
  MOZ_CRASH();
}
void MacroAssembler::cmp32LoadPtr(Condition,
                                  const Address&,
                                  Imm32,
                                  const Address&,
                                  Register) {
  MOZ_CRASH();
}
void MacroAssembler::cmp32Move32(Condition cond,
                                 Register lhs,
                                 Register rhs,
                                 Register src,
                                 Register dest) {
  UseScratchRegisterScope temps(this);
  Register scratch2 = temps.Acquire();
  cmp32Set(cond, lhs, rhs, scratch2);
  moveIfNotZero(dest, src, scratch2);
}

void MacroAssembler::cmp32Move32(Condition cond,
                                 Register lhs,
                                 const Address& rhs,
                                 Register src,
                                 Register dest) {
  UseScratchRegisterScope temps(this);
  Register scratch2 = temps.Acquire();
  MOZ_ASSERT(lhs != scratch2 && src != scratch2 && dest != scratch2);
  load32(rhs, scratch2);
  cmp32Move32(cond, lhs, scratch2, src, dest);
}
void MacroAssembler::cmp32MovePtr(Condition cond,
                                  Register lhs,
                                  Imm32 rhs,
                                  Register src,
                                  Register dest) {
  UseScratchRegisterScope temps(this);
  Register scratch2 = temps.Acquire();
  cmp32Set(cond, lhs, rhs, scratch2);
  moveIfNotZero(dest, src, scratch2);
}
void MacroAssembler::cmp64Set(Condition cond,
                              Address lhs,
                              Imm64 rhs,
                              Register dest) {
  ma_cmp_set(dest, lhs, ImmWord(uint64_t(rhs.value)), cond);
}
void MacroAssembler::cmp8Set(Condition, Address, Imm32, Register) {
  MOZ_CRASH();
}
void MacroAssembler::cmpPtrMovePtr(Condition cond,
                                   Register lhs,
                                   Register rhs,
                                   Register src,
                                   Register dest) {
  UseScratchRegisterScope temps(this);
  Register scratch2 = temps.Acquire();
  cmpPtrSet(cond, lhs, rhs, scratch2);
  moveIfNotZero(dest, src, scratch2);
}

void MacroAssembler::cmpPtrMovePtr(Condition cond,
                                   Register lhs,
                                   const Address& rhs,
                                   Register src,
                                   Register dest) {
  MOZ_CRASH("NYI");
}
void MacroAssembler::ctz32(Register, Register, bool) {
  MOZ_CRASH();
}
void MacroAssembler::ctz64(Register64, Register) {
  MOZ_CRASH();
}
void MacroAssembler::decBranchPtr(Condition, Register, Imm32, Label*) {
  MOZ_CRASH();
}
void MacroAssembler::divFloat32(FloatRegister src, FloatRegister dest) {
  fdiv_s(dest, dest, src);
}

void MacroAssembler::divDouble(FloatRegister src, FloatRegister dest) {
  fdiv_d(dest, dest, src);
}
void MacroAssembler::fallibleUnboxPtr(const ValueOperand& src,
                                      Register dest,
                                      JSValueType type,
                                      Label* fail) {
  MOZ_ASSERT(type == JSVAL_TYPE_OBJECT || type == JSVAL_TYPE_STRING ||
             type == JSVAL_TYPE_SYMBOL || type == JSVAL_TYPE_BIGINT);
  // dest := src XOR mask
  // scratch := dest >> JSVAL_TAG_SHIFT
  // fail if scratch != 0
  //
  // Note: src and dest can be the same register
  UseScratchRegisterScope temps(this);
  Register scratch = temps.Acquire();
  MOZ_ASSERT(src.valueReg() != scratch);
  mov(ImmWord(JSVAL_TYPE_TO_SHIFTED_TAG(type)), scratch);
  xor_(dest, src.valueReg(), scratch);
  srli(scratch, dest, JSVAL_TAG_SHIFT);
  ma_b(scratch, Imm32(0), fail, Assembler::NotEqual);
}

void MacroAssembler::fallibleUnboxPtr(const Address& src,
                                      Register dest,
                                      JSValueType type,
                                      Label* fail) {
  loadValue(src, ValueOperand(dest));
  fallibleUnboxPtr(ValueOperand(dest), dest, type, fail);
}

void MacroAssembler::fallibleUnboxPtr(const BaseIndex& src,
                                      Register dest,
                                      JSValueType type,
                                      Label* fail) {
  loadValue(src, ValueOperand(dest));
  fallibleUnboxPtr(ValueOperand(dest), dest, type, fail);
}
void MacroAssembler::flexibleLshift32(Register, Register) {
  MOZ_CRASH();
}
void MacroAssembler::flexibleRshift32Arithmetic(Register, Register) {
  MOZ_CRASH();
}
void MacroAssembler::flexibleRshift32(Register, Register) {
  MOZ_CRASH();
}
void MacroAssembler::inc64(AbsoluteAddress dest) {
  UseScratchRegisterScope temps(this);
  Register scratch = temps.Acquire();
  Register scratch2 = temps.Acquire();
  ma_li(scratch, ImmWord(uintptr_t(dest.addr)));
  ld(scratch2, scratch, 0);
  addi(scratch2, scratch2, 1);
  sd(scratch2, scratch, 0);
}

void MacroAssembler::load32SignExtendToPtr(const Address& src, Register dest) {
  load32(src, dest);
}
void MacroAssembler::loadAbiReturnAddress(Register dest) {
  movePtr(ra, dest);
}

void MacroAssembler::lshift32(Register src, Register dest) {
  sllw(dest, dest, src);
}

void MacroAssembler::lshift32(Imm32 imm, Register dest) {
  slliw(dest, dest, imm.value % 32);
}
void MacroAssembler::lshift64(Register shift, Register64 dest) {
  sll(dest.reg, dest.reg, shift);
}

void MacroAssembler::lshift64(Imm32 imm, Register64 dest) {
  MOZ_ASSERT(0 <= imm.value && imm.value < 64);
  slli(dest.reg, dest.reg, imm.value);
}
void MacroAssembler::lshiftPtr(Register shift, Register dest) {
  sll(dest, dest, shift);
}

void MacroAssembler::lshiftPtr(Imm32 imm, Register dest) {
  MOZ_ASSERT(0 <= imm.value && imm.value < 64);
  slli(dest, dest, imm.value);
}
void MacroAssembler::maxDouble(FloatRegister, FloatRegister, bool) {
  MOZ_CRASH();
}
void MacroAssembler::maxFloat32(FloatRegister, FloatRegister, bool) {
  MOZ_CRASH();
}
void MacroAssembler::memoryBarrier(MemoryBarrierBits) {
  MOZ_CRASH();
}
void MacroAssembler::minDouble(FloatRegister, FloatRegister, bool) {
  MOZ_CRASH();
}
void MacroAssembler::minFloat32(FloatRegister, FloatRegister, bool) {
  MOZ_CRASH();
}
void MacroAssembler::move16SignExtend(Register, Register) {
  MOZ_CRASH();
}
void MacroAssembler::move16To64SignExtend(Register, Register64) {
  MOZ_CRASH();
}
void MacroAssembler::move32SignExtendToPtr(Register, Register) {
  MOZ_CRASH();
}
void MacroAssembler::move32To64SignExtend(Register, Register64) {
  MOZ_CRASH();
}
void MacroAssembler::move32To64ZeroExtend(Register, Register64) {
  MOZ_CRASH();
}
void MacroAssembler::move32ZeroExtendToPtr(Register, Register) {
  MOZ_CRASH();
}
void MacroAssembler::move64(Imm64, Register64) {
  MOZ_CRASH();
}
void MacroAssembler::move64(Register64, Register64) {
  MOZ_CRASH();
}
void MacroAssembler::move64To32(Register64, Register) {
  MOZ_CRASH();
}
void MacroAssembler::move8SignExtend(Register, Register) {
  MOZ_CRASH();
}
void MacroAssembler::move8To64SignExtend(Register, Register64) {
  MOZ_CRASH();
}
void MacroAssembler::moveDoubleToGPR64(FloatRegister, Register64) {
  MOZ_CRASH();
}
void MacroAssembler::moveFloat32ToGPR(FloatRegister, Register) {
  MOZ_CRASH();
}
void MacroAssembler::moveGPR64ToDouble(Register64, FloatRegister) {
  MOZ_CRASH();
}
void MacroAssembler::moveGPRToFloat32(Register, FloatRegister) {
  MOZ_CRASH();
}
void MacroAssembler::mul32(Register rhs, Register srcDest) {
  mulw(srcDest, srcDest, rhs);
}

void MacroAssembler::mul32(Imm32 imm, Register srcDest) {
  ScratchRegisterScope scratch(asMasm());
  move32(imm, scratch);
  mul32(scratch, srcDest);
}
void MacroAssembler::mul64(Imm64 imm, const Register64& dest) {
  ScratchRegisterScope scratch(asMasm());
  MOZ_ASSERT(dest.reg != scratch);
  mov(ImmWord(imm.value), scratch);
  mul(dest.reg, dest.reg, scratch);
}

void MacroAssembler::mul64(Imm64 imm,
                           const Register64& dest,
                           const Register temp) {
  MOZ_ASSERT(temp == Register::Invalid());
  mul64(imm, dest);
}

void MacroAssembler::mul64(const Register64& src,
                           const Register64& dest,
                           const Register temp) {
  MOZ_ASSERT(temp == Register::Invalid());
  mul(dest.reg, dest.reg, src.reg);
}

void MacroAssembler::mul64(const Operand& src,
                           const Register64& dest,
                           const Register temp) {
  if (src.is_mem()) {
    ScratchRegisterScope scratch(asMasm());
    Register64 scratch64(scratch);

    load64(src.toAddress(), scratch64);
    mul64(scratch64, dest, temp);
  } else {
    mul64(Register64(src.toReg()), dest, temp);
  }
}
void MacroAssembler::mulBy3(Register src, Register dest) {
  ScratchRegisterScope scratch(asMasm());
  MOZ_ASSERT(src != scratch);
  add(scratch, src, src);
  add(dest, scratch, src);
}
void MacroAssembler::mulDouble(FloatRegister src, FloatRegister dest) {
  fmul_d(dest, dest, src);
}
void MacroAssembler::mulDoublePtr(ImmPtr imm,
                                  Register temp,
                                  FloatRegister dest) {
  ScratchRegisterScope scratch(asMasm());
  ScratchDoubleScope fpscratch(asMasm());
  movePtr(imm, scratch);
  loadDouble(Address(scratch, 0), fpscratch);
  mulDouble(fpscratch, dest);
}
void MacroAssembler::mulFloat32(FloatRegister src, FloatRegister dest) {
  fmul_s(dest, dest, src);
}
void MacroAssembler::mulPtr(Register rhs, Register srcDest) {
  mul(srcDest, srcDest, rhs);
}

void MacroAssembler::negateDouble(FloatRegister reg) {
  fneg_d(reg, reg);
}

void MacroAssembler::negateFloat(FloatRegister reg) {
  fneg_s(reg, reg);
}

void MacroAssembler::neg64(Register64 reg) {
  sub(reg.reg, zero, reg.reg);
}

void MacroAssembler::negPtr(Register reg) {
  sub(reg, zero, reg);
}

void MacroAssembler::neg32(Register reg) {
  subw(reg, zero, reg);
}
void MacroAssembler::not32(Register reg) {
  nor(reg, reg, zero);
}

void MacroAssembler::notPtr(Register reg) {
  nor(reg, reg, zero);
}

void MacroAssembler::or32(Register src, Register dest) {
  ma_or(dest, dest, src);
}

void MacroAssembler::or32(Imm32 imm, Register dest) {
  ma_or(dest, dest, imm);
}

void MacroAssembler::or32(Imm32 imm, const Address& dest) {
  UseScratchRegisterScope temps(this);
  Register scratch2 = temps.Acquire();
  load32(dest, scratch2);
  ma_or(scratch2, imm);
  store32(scratch2, dest);
}

void MacroAssembler::or64(Register64 src, Register64 dest) {
  ma_or(dest.reg, dest.reg, src.reg);
}

void MacroAssembler::or64(const Operand& src, Register64 dest) {
  if (src.is_mem()) {
    UseScratchRegisterScope temps(this);
    Register scratch = temps.Acquire();
    Register64 scratch64(scratch);

    load64(src.toAddress(), scratch64);
    or64(scratch64, dest);
  } else {
    or64(Register64(src.toReg()), dest);
  }
}
void MacroAssembler::or64(Imm64 imm, Register64 dest) {
  UseScratchRegisterScope temps(this);
  Register scratch = temps.Acquire();
  ma_li(scratch, ImmWord(imm.value));
  ma_or(dest.reg, dest.reg, scratch);
}

void MacroAssembler::orPtr(Register src, Register dest) {
  ma_or(dest, dest, src);
}

void MacroAssembler::orPtr(Imm32 imm, Register dest) {
  ma_or(dest, dest, imm);
}

void MacroAssembler::patchSub32FromStackPtr(CodeOffset, Imm32) {
  MOZ_CRASH();
}
void MacroAssembler::popcnt32(Register, Register, Register) {
  MOZ_CRASH();
}
void MacroAssembler::popcnt64(Register64, Register64, Register) {
  MOZ_CRASH();
}
void MacroAssembler::quotient32(Register rhs, Register srcDest,
                                bool isUnsigned) {
  if (isUnsigned) {
    ma_divu32(srcDest, srcDest, rhs);
  } else {
    ma_div32(srcDest, srcDest, rhs);
  }
}

void MacroAssembler::remainder32(Register rhs, Register srcDest,
                                 bool isUnsigned) {
  if (isUnsigned) {
    ma_modu32(srcDest, srcDest, rhs);
  } else {
    ma_mod32(srcDest, srcDest, rhs);
  }
}
void MacroAssembler::rotateLeft64(Imm32, Register64, Register64, Register) {
  MOZ_CRASH();
}
void MacroAssembler::rotateLeft64(Register, Register64, Register64, Register) {
  MOZ_CRASH();
}
void MacroAssembler::rotateLeft(Imm32, Register, Register) {
  MOZ_CRASH();
}
void MacroAssembler::rotateLeft(Register, Register, Register) {
  MOZ_CRASH();
}
void MacroAssembler::rotateRight64(Imm32, Register64, Register64, Register) {
  MOZ_CRASH();
}
void MacroAssembler::rotateRight64(Register, Register64, Register64, Register) {
  MOZ_CRASH();
}
void MacroAssembler::rotateRight(Imm32, Register, Register) {
  MOZ_CRASH();
}
void MacroAssembler::rotateRight(Register, Register, Register) {
  MOZ_CRASH();
}
void MacroAssembler::rshift32Arithmetic(Register src, Register dest) {
  sraw(dest, dest, src);
}

void MacroAssembler::rshift32Arithmetic(Imm32 imm, Register dest) {
  sraiw(dest, dest, imm.value % 32);
}
void MacroAssembler::rshift32(Register src, Register dest) {
  srlw(dest, dest, src);
}

void MacroAssembler::rshift32(Imm32 imm, Register dest) {
  srliw(dest, dest, imm.value % 32);
}

void MacroAssembler::rshift64Arithmetic(Imm32 imm, Register64 dest) {
  MOZ_ASSERT(0 <= imm.value && imm.value < 64);
  srai(dest.reg, dest.reg, imm.value);
}

void MacroAssembler::rshift64Arithmetic(Register shift, Register64 dest) {
  sra(dest.reg, dest.reg, shift);
}

void MacroAssembler::rshift64(Register shift, Register64 dest) {
  srl(dest.reg, dest.reg, shift);
}

void MacroAssembler::rshift64(Imm32 imm, Register64 dest) {
  MOZ_ASSERT(0 <= imm.value && imm.value < 64);
  srli(dest.reg, dest.reg, imm.value);
}

void MacroAssembler::rshiftPtrArithmetic(Imm32 imm, Register dest) {
  MOZ_ASSERT(0 <= imm.value && imm.value < 64);
  srai(dest, dest, imm.value);
}
void MacroAssembler::rshiftPtr(Register shift, Register dest) {
  srl(dest, dest, shift);
}

void MacroAssembler::rshiftPtr(Imm32 imm, Register dest) {
  MOZ_ASSERT(0 <= imm.value && imm.value < 64);
  srli(dest, dest, imm.value);
}
void MacroAssembler::spectreBoundsCheck32(Register index,
                                          Register length,
                                          Register maybeScratch,
                                          Label* failure) {
  MOZ_RELEASE_ASSERT(!JitOptions.spectreIndexMasking);
  branch32(Assembler::BelowOrEqual, length, index, failure);
}

void MacroAssembler::spectreBoundsCheck32(Register index,
                                          const Address& length,
                                          Register maybeScratch,
                                          Label* failure) {
  MOZ_RELEASE_ASSERT(!JitOptions.spectreIndexMasking);
  branch32(Assembler::BelowOrEqual, length, index, failure);
}
void MacroAssembler::spectreBoundsCheckPtr(Register index,
                                           Register length,
                                           Register maybeScratch,
                                           Label* failure) {
  MOZ_RELEASE_ASSERT(!JitOptions.spectreIndexMasking);
  branchPtr(Assembler::BelowOrEqual, length, index, failure);
}

void MacroAssembler::spectreBoundsCheckPtr(Register index,
                                           const Address& length,
                                           Register maybeScratch,
                                           Label* failure) {
  MOZ_RELEASE_ASSERT(!JitOptions.spectreIndexMasking);
  branchPtr(Assembler::BelowOrEqual, length, index, failure);
}
void MacroAssembler::spectreMovePtr(Condition, Register, Register) {
  MOZ_CRASH();
}
void MacroAssembler::spectreZeroRegister(Condition, Register, Register) {
  MOZ_CRASH();
}
void MacroAssembler::sqrtDouble(FloatRegister src, FloatRegister dest) {
  fsqrt_d(dest, src);
}
void MacroAssembler::sqrtFloat32(FloatRegister src, FloatRegister dest) {
  fsqrt_s(dest, src);
}
void MacroAssembler::storeUncanonicalizedFloat32(FloatRegister src,
                                                 const Address& addr) {
  ma_fst_s(src, addr);
}
void MacroAssembler::storeUncanonicalizedFloat32(FloatRegister src,
                                                 const BaseIndex& addr) {
  ma_fst_s(src, addr);
}

void MacroAssembler::storeUncanonicalizedDouble(FloatRegister src,
                                                const Address& addr) {
  ma_fst_d(src, addr);
}
void MacroAssembler::storeUncanonicalizedDouble(FloatRegister src,
                                                const BaseIndex& addr) {
  ma_fst_d(src, addr);
}
void MacroAssembler::sub32(Register src, Register dest) {
  subw(dest, dest, src);
}

void MacroAssembler::sub32(Imm32 imm, Register dest) {
  ma_sub32(dest, dest, imm);
}

void MacroAssembler::sub32(const Address& src, Register dest) {
  UseScratchRegisterScope temps(this);
  Register scratch = temps.Acquire();
  load32(src, scratch);
  subw(dest, dest, scratch);
}

void MacroAssembler::sub64(Register64 src, Register64 dest) {
  sub(dest.reg, dest.reg, src.reg);
}

void MacroAssembler::sub64(const Operand& src, Register64 dest) {
  if (src.is_mem()) {
    UseScratchRegisterScope temps(this);
    Register scratch = temps.Acquire();
    Register64 scratch64(scratch);

    load64(src.toAddress(), scratch64);
    sub64(scratch64, dest);
  } else {
    sub64(Register64(src.toReg()), dest);
  }
}

void MacroAssembler::sub64(Imm64 imm, Register64 dest) {
  UseScratchRegisterScope temps(this);
  Register scratch = temps.Acquire();
  MOZ_ASSERT(dest.reg != scratch);
  ma_li(scratch, ImmWord(imm.value));
  sub(dest.reg, dest.reg, scratch);
}

void MacroAssembler::subDouble(FloatRegister src, FloatRegister dest) {
  fsub_d(dest, dest, src);
}

void MacroAssembler::subFloat32(FloatRegister src, FloatRegister dest) {
  fsub_s(dest, dest, src);
}

void MacroAssembler::subPtr(Register src, const Address& dest) {
  UseScratchRegisterScope temps(this);
  Register scratch = temps.Acquire();
  loadPtr(dest, scratch);
  subPtr(src, scratch);
  storePtr(scratch, dest);
}

void MacroAssembler::subPtr(const Address& addr, Register dest) {
  UseScratchRegisterScope temps(this);
  Register scratch = temps.Acquire();
  loadPtr(addr, scratch);
  subPtr(scratch, dest);
}
void MacroAssembler::subPtr(Imm32 imm, Register dest) {
  ma_sub64(dest, dest, imm);
}
void MacroAssembler::subPtr(Register src, Register dest) {
  sub(dest, dest, src);
}
void MacroAssembler::test32LoadPtr(Condition cond,
                                   const Address& addr,
                                   Imm32 mask,
                                   const Address& src,
                                   Register dest) {
  MOZ_RELEASE_ASSERT(!JitOptions.spectreStringMitigations);
  Label skip;
  branchTest32(Assembler::InvertCondition(cond), addr, mask, &skip);
  loadPtr(src, dest);
  bind(&skip);
}
void MacroAssembler::test32MovePtr(Condition,
                                   const Address&,
                                   Imm32,
                                   Register,
                                   Register) {
  MOZ_CRASH();
}
void MacroAssembler::xor32(Register src, Register dest) {
  ma_xor(dest, dest, src);
}

void MacroAssembler::xor32(Imm32 imm, Register dest) {
  ma_xor(dest, dest, imm);
}

void MacroAssembler::xor32(Imm32 imm, const Address& dest) {
  UseScratchRegisterScope temps(this);
  Register scratch2 = temps.Acquire();
  load32(dest, scratch2);
  xor32(imm, scratch2);
  store32(scratch2, dest);
}

void MacroAssembler::xor32(const Address& src, Register dest) {
  UseScratchRegisterScope temps(this);
  Register scratch2 = temps.Acquire();
  load32(src, scratch2);
  xor32(scratch2, dest);
}
void MacroAssembler::xor64(Register64 src, Register64 dest) {
  ma_xor(dest.reg, dest.reg, src.reg);
}

void MacroAssembler::xor64(const Operand& src, Register64 dest) {
  if (src.is_mem()) {
    UseScratchRegisterScope temps(this);
    Register scratch = temps.Acquire();
    Register64 scratch64(scratch);

    load64(src.toAddress(), scratch64);
    xor64(scratch64, dest);
  } else {
    xor64(Register64(src.toReg()), dest);
  }
}
void MacroAssembler::xor64(Imm64 imm, Register64 dest) {
  UseScratchRegisterScope temps(this);
  Register scratch = temps.Acquire();
  ma_li(scratch, ImmWord(imm.value));
  ma_xor(dest.reg, dest.reg, scratch);
}
void MacroAssembler::xorPtr(Register src, Register dest) {
  ma_xor(dest, dest, src);
}

void MacroAssembler::xorPtr(Imm32 imm, Register dest) {
  ma_xor(dest, dest, imm);
}
//}}} check_macroassembler_style

void MacroAssemblerRiscv64Compat::incrementInt32Value(const Address& addr) {
  asMasm().add32(Imm32(1), addr);
}

void MacroAssemblerRiscv64Compat::retn(Imm32 n) {
  // pc <- [sp]; sp += n
  loadPtr(Address(StackPointer, 0), ra);
  asMasm().addPtr(n, StackPointer);
  jr(ra, 0);
}
}  // namespace jit
}  // namespace js

#endif /* jit_riscv64_MacroAssembler_riscv64_inl_h */
