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

//{{{ check_macroassembler_style
CodeOffset MacroAssembler::sub32FromStackPtrWithPatch(Register) {
  MOZ_CRASH();
}
void MacroAssembler::mul64(const Operand& src, const Register64& dest, const Register temp) {
  MOZ_CRASH();
}
void MacroAssembler::mul64(const Register64& src,
                           const Register64& dest,
                           const Register temp) {
  MOZ_CRASH();
}
void MacroAssembler::mul64(Imm64 imm, const Register64& dest, const Register tmp) {
  MOZ_CRASH();
}
template <class L>
void MacroAssembler::branch32(Condition, Register, Imm32, L) {
  MOZ_CRASH();
}
template <class L>
void MacroAssembler::branch32(Condition, Register, Register, L) {
  MOZ_CRASH();
}
template <class L>
void MacroAssembler::branchPtr(Condition, const Address&, Register, L) {
  MOZ_CRASH();
}
template <class L>
void MacroAssembler::branchPtr(Condition, Register, Register, L) {
  MOZ_CRASH();
}
template <class L>
void MacroAssembler::branchTest32(Condition, Register, Imm32, L) {
  MOZ_CRASH();
}
template <class L>
void MacroAssembler::branchTest32(Condition, Register, Register, L) {
  MOZ_CRASH();
}
template <class L>
void MacroAssembler::branchTest64(Condition,
                                  Register64,
                                  Register64,
                                  Register,
                                  L) {
  MOZ_CRASH();
}
template <class L>
void MacroAssembler::branchTestMagic(Condition, const ValueOperand&, L) {
  MOZ_CRASH();
}
template <class L>
void MacroAssembler::branchTestPtr(Condition, Register, Register, L) {
  MOZ_CRASH();
}
template <typename T>
void MacroAssembler::branchAdd32(Condition, T, Register, Label*) {
  MOZ_CRASH();
}
template <typename T>
void MacroAssembler::branchAddPtr(Condition, T, Register, Label*) {
  MOZ_CRASH();
}
template <typename T>
void MacroAssembler::branchMul32(Condition, T, Register, Label*) {
  MOZ_CRASH();
}
template <typename T>
void MacroAssembler::branchRshift32(Condition, T, Register, Label*) {
  MOZ_CRASH();
}
template <typename T>
void MacroAssembler::branchSub32(Condition, T, Register, Label*) {
  MOZ_CRASH();
}
template <typename T>
void MacroAssembler::branchSubPtr(Condition, T, Register, Label*) {
  MOZ_CRASH();
}
template <typename T>
void MacroAssembler::branchTestGCThingImpl(Condition, const T&, Label*) {
  MOZ_CRASH();
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
template <typename T1, typename T2>
void MacroAssembler::cmp32Set(Condition, T1, T2, Register) {
  MOZ_CRASH();
}
template <typename T1, typename T2>
void MacroAssembler::cmpPtrSet(Condition, T1, T2, Register) {
  MOZ_CRASH();
}
void MacroAssembler::abs32(Register, Register) {
  MOZ_CRASH();
}
void MacroAssembler::absDouble(FloatRegister, FloatRegister) {
  MOZ_CRASH();
}
void MacroAssembler::absFloat32(FloatRegister, FloatRegister) {
  MOZ_CRASH();
}
void MacroAssembler::add32(Imm32, const Address&) {
  MOZ_CRASH();
}
void MacroAssembler::add32(Imm32, Register) {
  MOZ_CRASH();
}
void MacroAssembler::add32(Register, Register) {
  MOZ_CRASH();
}
void MacroAssembler::add64(const Operand&, Register64) {
  MOZ_CRASH();
}
void MacroAssembler::add64(Imm32, Register64) {
  MOZ_CRASH();
}
void MacroAssembler::add64(Imm64, Register64) {
  MOZ_CRASH();
}
void MacroAssembler::add64(Register64, Register64) {
  MOZ_CRASH();
}
void MacroAssembler::addDouble(FloatRegister, FloatRegister) {
  MOZ_CRASH();
}
void MacroAssembler::addFloat32(FloatRegister, FloatRegister) {
  MOZ_CRASH();
}
void MacroAssembler::addPtr(const Address&, Register) {
  MOZ_CRASH();
}
void MacroAssembler::addPtr(Imm32, const Address&) {
  MOZ_CRASH();
}
void MacroAssembler::addPtr(Imm32, Register) {
  MOZ_CRASH();
}
void MacroAssembler::addPtr(ImmWord, Register) {
  MOZ_CRASH();
}
void MacroAssembler::addPtr(Register, Register) {
  MOZ_CRASH();
}
void MacroAssembler::and32(const Address&, Register) {
  MOZ_CRASH();
}
void MacroAssembler::and32(Imm32, const Address&) {
  MOZ_CRASH();
}
void MacroAssembler::and32(Imm32, Register) {
  MOZ_CRASH();
}
void MacroAssembler::and32(Register, Register) {
  MOZ_CRASH();
}
void MacroAssembler::and64(const Operand&, Register64) {
  MOZ_CRASH();
}
void MacroAssembler::and64(Imm64, Register64) {
  MOZ_CRASH();
}
void MacroAssembler::and64(Register64, Register64) {
  MOZ_CRASH();
}
void MacroAssembler::andPtr(Imm32, Register) {
  MOZ_CRASH();
}
void MacroAssembler::andPtr(Register, Register) {
  MOZ_CRASH();
}
void MacroAssembler::branch16(Condition, const Address&, Imm32, Label*) {
  MOZ_CRASH();
}
void MacroAssembler::branch32(Condition,
                              const AbsoluteAddress&,
                              Imm32,
                              Label*) {
  MOZ_CRASH();
}
void MacroAssembler::branch32(Condition,
                              const AbsoluteAddress&,
                              Register,
                              Label*) {
  MOZ_CRASH();
}
void MacroAssembler::branch32(Condition, const Address&, Imm32, Label*) {
  MOZ_CRASH();
}
void MacroAssembler::branch32(Condition, const Address&, Register, Label*) {
  MOZ_CRASH();
}
void MacroAssembler::branch32(Condition, const BaseIndex&, Imm32, Label*) {
  MOZ_CRASH();
}
void MacroAssembler::branch32(Condition, wasm::SymbolicAddress, Imm32, Label*) {
  MOZ_CRASH();
}
void MacroAssembler::branch64(Condition,
                              const Address&,
                              const Address&,
                              Register,
                              Label*) {
  MOZ_CRASH();
}
void MacroAssembler::branch64(Condition, const Address&, Imm64, Label*) {
  MOZ_CRASH();
}
void MacroAssembler::branch64(Condition, const Address&, Register64, Label*) {
  MOZ_CRASH();
}
void MacroAssembler::branch64(Condition, Register64, Imm64, Label*, Label*) {
  MOZ_CRASH();
}
void MacroAssembler::branch64(Condition,
                              Register64,
                              Register64,
                              Label*,
                              Label*) {
  MOZ_CRASH();
}
void MacroAssembler::branch8(Condition, const Address&, Imm32, Label*) {
  MOZ_CRASH();
}
void MacroAssembler::branchDouble(DoubleCondition,
                                  FloatRegister,
                                  FloatRegister,
                                  Label*) {
  MOZ_CRASH();
}
void MacroAssembler::branchFloat(DoubleCondition,
                                 FloatRegister,
                                 FloatRegister,
                                 Label*) {
  MOZ_CRASH();
}
void MacroAssembler::branchMulPtr(Condition, Register, Register, Label*) {
  MOZ_CRASH();
}
void MacroAssembler::branchNeg32(Condition, Register, Label*) {
  MOZ_CRASH();
}
void MacroAssembler::branchPrivatePtr(Condition,
                                      const Address&,
                                      Register,
                                      Label*) {
  MOZ_CRASH();
}
void MacroAssembler::branchPtr(Condition,
                               const AbsoluteAddress&,
                               ImmWord,
                               Label*) {
  MOZ_CRASH();
}
void MacroAssembler::branchPtr(Condition,
                               const AbsoluteAddress&,
                               Register,
                               Label*) {
  MOZ_CRASH();
}
void MacroAssembler::branchPtr(Condition, const Address&, ImmGCPtr, Label*) {
  MOZ_CRASH();
}
void MacroAssembler::branchPtr(Condition, const Address&, ImmPtr, Label*) {
  MOZ_CRASH();
}
void MacroAssembler::branchPtr(Condition, const Address&, ImmWord, Label*) {
  MOZ_CRASH();
}
void MacroAssembler::branchPtr(Condition, const BaseIndex&, ImmWord, Label*) {
  MOZ_CRASH();
}
void MacroAssembler::branchPtr(Condition, const BaseIndex&, Register, Label*) {
  MOZ_CRASH();
}
void MacroAssembler::branchPtr(Condition, Register, Imm32, Label*) {
  MOZ_CRASH();
}
void MacroAssembler::branchPtr(Condition, Register, ImmGCPtr, Label*) {
  MOZ_CRASH();
}
void MacroAssembler::branchPtr(Condition, Register, ImmPtr, Label*) {
  MOZ_CRASH();
}
void MacroAssembler::branchPtr(Condition, Register, ImmWord, Label*) {
  MOZ_CRASH();
}
void MacroAssembler::branchPtr(Condition,
                               wasm::SymbolicAddress,
                               Register,
                               Label*) {
  MOZ_CRASH();
}
void MacroAssembler::branchTest32(Condition,
                                  const AbsoluteAddress&,
                                  Imm32,
                                  Label*) {
  MOZ_CRASH();
}
void MacroAssembler::branchTest32(Condition, const Address&, Imm32, Label*) {
  MOZ_CRASH();
}
void MacroAssembler::branchTestBigInt(Condition, const Address&, Label*) {
  MOZ_CRASH();
}
void MacroAssembler::branchTestBigInt(Condition, const BaseIndex&, Label*) {
  MOZ_CRASH();
}
void MacroAssembler::branchTestBigInt(Condition, const ValueOperand&, Label*) {
  MOZ_CRASH();
}
void MacroAssembler::branchTestBigInt(Condition, Register, Label*) {
  MOZ_CRASH();
}
void MacroAssembler::branchTestBigIntTruthy(bool, const ValueOperand&, Label*) {
  MOZ_CRASH();
}
void MacroAssembler::branchTestBoolean(Condition, const Address&, Label*) {
  MOZ_CRASH();
}
void MacroAssembler::branchTestBoolean(Condition, const BaseIndex&, Label*) {
  MOZ_CRASH();
}
void MacroAssembler::branchTestBoolean(Condition, const ValueOperand&, Label*) {
  MOZ_CRASH();
}
void MacroAssembler::branchTestBoolean(Condition, Register, Label*) {
  MOZ_CRASH();
}
void MacroAssembler::branchTestBooleanTruthy(bool,
                                             const ValueOperand&,
                                             Label*) {
  MOZ_CRASH();
}
void MacroAssembler::branchTestDouble(Condition, const Address&, Label*) {
  MOZ_CRASH();
}
void MacroAssembler::branchTestDouble(Condition, const BaseIndex&, Label*) {
  MOZ_CRASH();
}
void MacroAssembler::branchTestDouble(Condition, const ValueOperand&, Label*) {
  MOZ_CRASH();
}
void MacroAssembler::branchTestDouble(Condition, Register, Label*) {
  MOZ_CRASH();
}
void MacroAssembler::branchTestDoubleTruthy(bool, FloatRegister, Label*) {
  MOZ_CRASH();
}
void MacroAssembler::branchTestGCThing(Condition, const Address&, Label*) {
  MOZ_CRASH();
}
void MacroAssembler::branchTestGCThing(Condition, const BaseIndex&, Label*) {
  MOZ_CRASH();
}
void MacroAssembler::branchTestGCThing(Condition, const ValueOperand&, Label*) {
  MOZ_CRASH();
}
void MacroAssembler::branchTestInt32(Condition, const Address&, Label*) {
  MOZ_CRASH();
}
void MacroAssembler::branchTestInt32(Condition, const BaseIndex&, Label*) {
  MOZ_CRASH();
}
void MacroAssembler::branchTestInt32(Condition, const ValueOperand&, Label*) {
  MOZ_CRASH();
}
void MacroAssembler::branchTestInt32(Condition, Register, Label*) {
  MOZ_CRASH();
}
void MacroAssembler::branchTestInt32Truthy(bool, const ValueOperand&, Label*) {
  MOZ_CRASH();
}
void MacroAssembler::branchTestMagic(Condition,
                                     const Address&,
                                     JSWhyMagic,
                                     Label*) {
  MOZ_CRASH();
}
void MacroAssembler::branchTestMagic(Condition, const Address&, Label*) {
  MOZ_CRASH();
}
void MacroAssembler::branchTestMagic(Condition, const BaseIndex&, Label*) {
  MOZ_CRASH();
}
void MacroAssembler::branchTestMagic(Condition, Register, Label*) {
  MOZ_CRASH();
}
void MacroAssembler::branchTestNull(Condition, const Address&, Label*) {
  MOZ_CRASH();
}
void MacroAssembler::branchTestNull(Condition, const BaseIndex&, Label*) {
  MOZ_CRASH();
}
void MacroAssembler::branchTestNull(Condition, const ValueOperand&, Label*) {
  MOZ_CRASH();
}
void MacroAssembler::branchTestNull(Condition, Register, Label*) {
  MOZ_CRASH();
}
void MacroAssembler::branchTestNumber(Condition, const ValueOperand&, Label*) {
  MOZ_CRASH();
}
void MacroAssembler::branchTestNumber(Condition, Register, Label*) {
  MOZ_CRASH();
}
void MacroAssembler::branchTestObject(Condition, const Address&, Label*) {
  MOZ_CRASH();
}
void MacroAssembler::branchTestObject(Condition, const BaseIndex&, Label*) {
  MOZ_CRASH();
}
void MacroAssembler::branchTestObject(Condition, const ValueOperand&, Label*) {
  MOZ_CRASH();
}
void MacroAssembler::branchTestObject(Condition, Register, Label*) {
  MOZ_CRASH();
}
void MacroAssembler::branchTestPrimitive(Condition,
                                         const ValueOperand&,
                                         Label*) {
  MOZ_CRASH();
}
void MacroAssembler::branchTestPrimitive(Condition, Register, Label*) {
  MOZ_CRASH();
}
void MacroAssembler::branchTestPtr(Condition, const Address&, Imm32, Label*) {
  MOZ_CRASH();
}
void MacroAssembler::branchTestPtr(Condition, Register, Imm32, Label*) {
  MOZ_CRASH();
}
void MacroAssembler::branchTestString(Condition, const Address&, Label*) {
  MOZ_CRASH();
}
void MacroAssembler::branchTestString(Condition, const BaseIndex&, Label*) {
  MOZ_CRASH();
}
void MacroAssembler::branchTestString(Condition, const ValueOperand&, Label*) {
  MOZ_CRASH();
}
void MacroAssembler::branchTestString(Condition, Register, Label*) {
  MOZ_CRASH();
}
void MacroAssembler::branchTestStringTruthy(bool, const ValueOperand&, Label*) {
  MOZ_CRASH();
}
void MacroAssembler::branchTestSymbol(Condition, const Address&, Label*) {
  MOZ_CRASH();
}
void MacroAssembler::branchTestSymbol(Condition, const BaseIndex&, Label*) {
  MOZ_CRASH();
}
void MacroAssembler::branchTestSymbol(Condition, const ValueOperand&, Label*) {
  MOZ_CRASH();
}
void MacroAssembler::branchTestSymbol(Condition, Register, Label*) {
  MOZ_CRASH();
}
void MacroAssembler::branchTestUndefined(Condition, const Address&, Label*) {
  MOZ_CRASH();
}
void MacroAssembler::branchTestUndefined(Condition, const BaseIndex&, Label*) {
  MOZ_CRASH();
}
void MacroAssembler::branchTestUndefined(Condition,
                                         const ValueOperand&,
                                         Label*) {
  MOZ_CRASH();
}
void MacroAssembler::branchTestUndefined(Condition, Register, Label*) {
  MOZ_CRASH();
}
void MacroAssembler::branchTestValue(Condition,
                                     const BaseIndex&,
                                     const ValueOperand&,
                                     Label*) {
  MOZ_CRASH();
}
void MacroAssembler::branchToComputedAddress(const BaseIndex&) {
  MOZ_CRASH();
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
void MacroAssembler::cmp32Move32(Condition,
                                 Register,
                                 const Address&,
                                 Register,
                                 Register) {
  MOZ_CRASH();
}
void MacroAssembler::cmp32Move32(Condition,
                                 Register,
                                 Register,
                                 Register,
                                 Register) {
  MOZ_CRASH();
}
void MacroAssembler::cmp32MovePtr(Condition,
                                  Register,
                                  Imm32,
                                  Register,
                                  Register) {
  MOZ_CRASH();
}
void MacroAssembler::cmp64Set(Condition, Address, Imm64, Register) {
  MOZ_CRASH();
}
void MacroAssembler::cmp8Set(Condition, Address, Imm32, Register) {
  MOZ_CRASH();
}
void MacroAssembler::cmpPtrMovePtr(Condition,
                                   Register,
                                   const Address&,
                                   Register,
                                   Register) {
  MOZ_CRASH();
}
void MacroAssembler::cmpPtrMovePtr(Condition,
                                   Register,
                                   Register,
                                   Register,
                                   Register) {
  MOZ_CRASH();
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
void MacroAssembler::divDouble(FloatRegister, FloatRegister) {
  MOZ_CRASH();
}
void MacroAssembler::divFloat32(FloatRegister, FloatRegister) {
  MOZ_CRASH();
}
void MacroAssembler::fallibleUnboxPtr(const Address&,
                                      Register,
                                      JSValueType,
                                      Label*) {
  MOZ_CRASH();
}
void MacroAssembler::fallibleUnboxPtr(const BaseIndex&,
                                      Register,
                                      JSValueType,
                                      Label*) {
  MOZ_CRASH();
}
void MacroAssembler::fallibleUnboxPtr(const ValueOperand&,
                                      Register,
                                      JSValueType,
                                      Label*) {
  MOZ_CRASH();
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
void MacroAssembler::inc64(AbsoluteAddress) {
  MOZ_CRASH();
}
void MacroAssembler::load32SignExtendToPtr(const Address&, Register) {
  MOZ_CRASH();
}
void MacroAssembler::loadAbiReturnAddress(Register) {
  MOZ_CRASH();
}
void MacroAssembler::lshift32(Imm32, Register) {
  MOZ_CRASH();
}
void MacroAssembler::lshift32(Register, Register) {
  MOZ_CRASH();
}
void MacroAssembler::lshift64(Imm32, Register64) {
  MOZ_CRASH();
}
void MacroAssembler::lshift64(Register, Register64) {
  MOZ_CRASH();
}
void MacroAssembler::lshiftPtr(Imm32, Register) {
  MOZ_CRASH();
}
void MacroAssembler::lshiftPtr(Register, Register) {
  MOZ_CRASH();
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
void MacroAssembler::mul32(Imm32, Register) {
  MOZ_CRASH();
}
void MacroAssembler::mul32(Register, Register) {
  MOZ_CRASH();
}
void MacroAssembler::mul64(Imm64, const Register64&) {
  MOZ_CRASH();
}
void MacroAssembler::mulBy3(Register, Register) {
  MOZ_CRASH();
}
void MacroAssembler::mulDouble(FloatRegister, FloatRegister) {
  MOZ_CRASH();
}
void MacroAssembler::mulDoublePtr(ImmPtr, Register, FloatRegister) {
  MOZ_CRASH();
}
void MacroAssembler::mulFloat32(FloatRegister, FloatRegister) {
  MOZ_CRASH();
}
void MacroAssembler::mulPtr(Register, Register) {
  MOZ_CRASH();
}
void MacroAssembler::neg32(Register) {
  MOZ_CRASH();
}
void MacroAssembler::neg64(Register64) {
  MOZ_CRASH();
}
void MacroAssembler::negateDouble(FloatRegister) {
  MOZ_CRASH();
}
void MacroAssembler::negateFloat(FloatRegister) {
  MOZ_CRASH();
}
void MacroAssembler::negPtr(Register) {
  MOZ_CRASH();
}
void MacroAssembler::not32(Register) {
  MOZ_CRASH();
}
void MacroAssembler::notPtr(Register) {
  MOZ_CRASH();
}
void MacroAssembler::or32(Imm32, const Address&) {
  MOZ_CRASH();
}
void MacroAssembler::or32(Imm32, Register) {
  MOZ_CRASH();
}
void MacroAssembler::or32(Register, Register) {
  MOZ_CRASH();
}
void MacroAssembler::or64(const Operand&, Register64) {
  MOZ_CRASH();
}
void MacroAssembler::or64(Imm64, Register64) {
  MOZ_CRASH();
}
void MacroAssembler::or64(Register64, Register64) {
  MOZ_CRASH();
}
void MacroAssembler::orPtr(Imm32, Register) {
  MOZ_CRASH();
}
void MacroAssembler::orPtr(Register, Register) {
  MOZ_CRASH();
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
void MacroAssembler::quotient32(Register, Register, bool) {
  MOZ_CRASH();
}
void MacroAssembler::remainder32(Register, Register, bool) {
  MOZ_CRASH();
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
void MacroAssembler::rshift32Arithmetic(Imm32, Register) {
  MOZ_CRASH();
}
void MacroAssembler::rshift32Arithmetic(Register, Register) {
  MOZ_CRASH();
}
void MacroAssembler::rshift32(Imm32, Register) {
  MOZ_CRASH();
}
void MacroAssembler::rshift32(Register, Register) {
  MOZ_CRASH();
}
void MacroAssembler::rshift64Arithmetic(Imm32, Register64) {
  MOZ_CRASH();
}
void MacroAssembler::rshift64Arithmetic(Register, Register64) {
  MOZ_CRASH();
}
void MacroAssembler::rshift64(Imm32, Register64) {
  MOZ_CRASH();
}
void MacroAssembler::rshift64(Register, Register64) {
  MOZ_CRASH();
}
void MacroAssembler::rshiftPtrArithmetic(Imm32, Register) {
  MOZ_CRASH();
}
void MacroAssembler::rshiftPtr(Imm32, Register) {
  MOZ_CRASH();
}
void MacroAssembler::rshiftPtr(Register, Register) {
  MOZ_CRASH();
}
void MacroAssembler::spectreBoundsCheck32(Register,
                                          const Address&,
                                          Register,
                                          Label*) {
  MOZ_CRASH();
}
void MacroAssembler::spectreBoundsCheck32(Register,
                                          Register,
                                          Register,
                                          Label*) {
  MOZ_CRASH();
}
void MacroAssembler::spectreBoundsCheckPtr(Register,
                                           const Address&,
                                           Register,
                                           Label*) {
  MOZ_CRASH();
}
void MacroAssembler::spectreBoundsCheckPtr(Register,
                                           Register,
                                           Register,
                                           Label*) {
  MOZ_CRASH();
}
void MacroAssembler::spectreMovePtr(Condition, Register, Register) {
  MOZ_CRASH();
}
void MacroAssembler::spectreZeroRegister(Condition, Register, Register) {
  MOZ_CRASH();
}
void MacroAssembler::sqrtDouble(FloatRegister, FloatRegister) {
  MOZ_CRASH();
}
void MacroAssembler::sqrtFloat32(FloatRegister, FloatRegister) {
  MOZ_CRASH();
}
void MacroAssembler::storeUncanonicalizedDouble(FloatRegister, const Address&) {
  MOZ_CRASH();
}
void MacroAssembler::storeUncanonicalizedDouble(FloatRegister,
                                                const BaseIndex&) {
  MOZ_CRASH();
}
void MacroAssembler::storeUncanonicalizedFloat32(FloatRegister,
                                                 const Address&) {
  MOZ_CRASH();
}
void MacroAssembler::storeUncanonicalizedFloat32(FloatRegister,
                                                 const BaseIndex&) {
  MOZ_CRASH();
}
void MacroAssembler::sub32(const Address&, Register) {
  MOZ_CRASH();
}
void MacroAssembler::sub32(Imm32, Register) {
  MOZ_CRASH();
}
void MacroAssembler::sub32(Register, Register) {
  MOZ_CRASH();
}
void MacroAssembler::sub64(const Operand&, Register64) {
  MOZ_CRASH();
}
void MacroAssembler::sub64(Imm64, Register64) {
  MOZ_CRASH();
}
void MacroAssembler::sub64(Register64, Register64) {
  MOZ_CRASH();
}
void MacroAssembler::subDouble(FloatRegister, FloatRegister) {
  MOZ_CRASH();
}
void MacroAssembler::subFloat32(FloatRegister, FloatRegister) {
  MOZ_CRASH();
}
void MacroAssembler::subPtr(const Address&, Register) {
  MOZ_CRASH();
}
void MacroAssembler::subPtr(Imm32, Register) {
  MOZ_CRASH();
}
void MacroAssembler::subPtr(Register, const Address&) {
  MOZ_CRASH();
}
void MacroAssembler::subPtr(Register, Register) {
  MOZ_CRASH();
}
void MacroAssembler::test32LoadPtr(Condition,
                                   const Address&,
                                   Imm32,
                                   const Address&,
                                   Register) {
  MOZ_CRASH();
}
void MacroAssembler::test32MovePtr(Condition,
                                   const Address&,
                                   Imm32,
                                   Register,
                                   Register) {
  MOZ_CRASH();
}
void MacroAssembler::xor32(const Address&, Register) {
  MOZ_CRASH();
}
void MacroAssembler::xor32(Imm32, const Address&) {
  MOZ_CRASH();
}
void MacroAssembler::xor32(Imm32, Register) {
  MOZ_CRASH();
}
void MacroAssembler::xor32(Register, Register) {
  MOZ_CRASH();
}
void MacroAssembler::xor64(const Operand&, Register64) {
  MOZ_CRASH();
}
void MacroAssembler::xor64(Imm64, Register64) {
  MOZ_CRASH();
}
void MacroAssembler::xor64(Register64, Register64) {
  MOZ_CRASH();
}
void MacroAssembler::xorPtr(Imm32, Register) {
  MOZ_CRASH();
}
void MacroAssembler::xorPtr(Register, Register) {
  MOZ_CRASH();
}
//}}} check_macroassembler_style

}  // namespace jit
}  // namespace js

#endif /* jit_riscv64_MacroAssembler_riscv64_inl_h */
