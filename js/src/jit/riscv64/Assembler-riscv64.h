/* -*- Mode: C++; tab-width: 8; indent-tabs-mode: nil; c-basic-offset: 2 -*-
 * vim: set ts=8 sts=2 et sw=2 tw=80:
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */

#ifndef jit_riscv64_Assembler_riscv64_h
#define jit_riscv64_Assembler_riscv64_h

#include "mozilla/Assertions.h"

#include <stdint.h>

#include "jit/riscv64/Architecture-riscv64.h"
#include "jit/Registers.h"
#include "jit/RegisterSets.h"
#include "jit/shared/Assembler-shared.h"

namespace js {
namespace jit {


static constexpr Register zero{Registers::zero};
static constexpr Register ra{Registers::ra};
static constexpr Register tp{Registers::tp};
static constexpr Register sp{Registers::sp};
static constexpr Register a0{Registers::a0};
static constexpr Register a1{Registers::a1};
static constexpr Register a2{Registers::a2};
static constexpr Register a3{Registers::a3};
static constexpr Register a4{Registers::a4};
static constexpr Register a5{Registers::a5};
static constexpr Register a6{Registers::a6};
static constexpr Register a7{Registers::a7};
static constexpr Register t0{Registers::t0};
static constexpr Register t1{Registers::t1};
static constexpr Register t2{Registers::t2};
static constexpr Register t3{Registers::t3};
static constexpr Register t4{Registers::t4};
static constexpr Register t5{Registers::t5};
static constexpr Register t6{Registers::t6};
static constexpr Register fp{Registers::fp};
static constexpr Register s1{Registers::s1};
static constexpr Register s2{Registers::s2};
static constexpr Register s3{Registers::s3};
static constexpr Register s4{Registers::s4};
static constexpr Register s5{Registers::s5};
static constexpr Register s6{Registers::s6};
static constexpr Register s7{Registers::s7};
static constexpr Register s8{Registers::s8};
static constexpr Register s9{Registers::s9};
static constexpr Register s10{Registers::s10};
static constexpr Register s11{Registers::s11};


static constexpr FloatRegister ft0{FloatRegisters::f0};
static constexpr FloatRegister ft1{FloatRegisters::f1};
static constexpr FloatRegister ft2{FloatRegisters::f2};
static constexpr FloatRegister ft3{FloatRegisters::f3};
static constexpr FloatRegister ft4{FloatRegisters::f4};
static constexpr FloatRegister ft5{FloatRegisters::f5};
static constexpr FloatRegister ft6{FloatRegisters::f6};
static constexpr FloatRegister ft7{FloatRegisters::f7};
static constexpr FloatRegister fs0{FloatRegisters::f8};
static constexpr FloatRegister fs1{FloatRegisters::f9};
static constexpr FloatRegister fa0{FloatRegisters::f10};
static constexpr FloatRegister fa1{FloatRegisters::f11};
static constexpr FloatRegister fa2{FloatRegisters::f12};
static constexpr FloatRegister fa3{FloatRegisters::f13};
static constexpr FloatRegister fa4{FloatRegisters::f14};
static constexpr FloatRegister fa5{FloatRegisters::f15};
static constexpr FloatRegister fa6{FloatRegisters::f16};
static constexpr FloatRegister fa7{FloatRegisters::f17};
static constexpr FloatRegister fs2{FloatRegisters::f18};
static constexpr FloatRegister fs3{FloatRegisters::f19};
static constexpr FloatRegister fs4{FloatRegisters::f20};
static constexpr FloatRegister fs5{FloatRegisters::f21};
static constexpr FloatRegister fs6{FloatRegisters::f22};
static constexpr FloatRegister fs7{FloatRegisters::f23};
static constexpr FloatRegister fs8{FloatRegisters::f24};
static constexpr FloatRegister fs9{FloatRegisters::f25};
static constexpr FloatRegister fs10{FloatRegisters::f26};
static constexpr FloatRegister fs11{FloatRegisters::f27};
static constexpr FloatRegister ft8{FloatRegisters::f28};
static constexpr FloatRegister ft9{FloatRegisters::f29};
static constexpr FloatRegister ft10{FloatRegisters::f30};
static constexpr FloatRegister ft11{FloatRegisters::f31};

class MacroAssembler;

static constexpr Register StackPointer{Registers::sp};
static constexpr Register FramePointer{Registers::fp};
static constexpr Register ReturnReg{Registers::a0};

static constexpr Register ScratchRegister{Registers::t5};
static constexpr Register SecondScratchReg{Registers::t6};

static constexpr FloatRegister ReturnFloat32Reg{FloatRegisters::fa0};
static constexpr FloatRegister ReturnDoubleReg{FloatRegisters::fa0};
static constexpr FloatRegister ReturnSimd128Reg{FloatRegisters::invalid_reg};
static constexpr FloatRegister ScratchSimd128Reg{FloatRegisters::invalid_reg};
static constexpr FloatRegister InvalidFloatReg{FloatRegisters::invalid_reg};

static constexpr FloatRegister ScratchFloat32Reg{FloatRegisters::ft10};
static constexpr FloatRegister ScratchDoubleReg{FloatRegisters::ft10};

struct ScratchFloat32Scope : public AutoFloatRegisterScope {
  explicit ScratchFloat32Scope(MacroAssembler& masm)
      : AutoFloatRegisterScope(masm, ScratchFloat32Reg) {}
};

struct ScratchDoubleScope : public AutoFloatRegisterScope {
  explicit ScratchDoubleScope(MacroAssembler& masm)
      : AutoFloatRegisterScope(masm, ScratchDoubleReg) {}
};

static constexpr Register OsrFrameReg{Registers::a3};
static constexpr Register PreBarrierReg{Registers::a1};
static constexpr Register InterpreterPCReg{Registers::t0};
static constexpr Register CallTempReg0{Registers::t0};
static constexpr Register CallTempReg1{Registers::t1};
static constexpr Register CallTempReg2{Registers::t2};
static constexpr Register CallTempReg3{Registers::t3};
static constexpr Register CallTempReg4{Registers::t4};
static constexpr Register CallTempReg5{Registers::t5};
static constexpr Register InvalidReg{Registers::invalid_reg};
static constexpr Register CallTempNonArgRegs[] = {t0, t1, t2, t3};
static const uint32_t NumCallTempNonArgRegs = std::size(CallTempNonArgRegs);

static constexpr Register IntArgReg0{Registers::a0};
static constexpr Register IntArgReg1{Registers::a1};
static constexpr Register IntArgReg2{Registers::a2};
static constexpr Register IntArgReg3{Registers::a3};
static constexpr Register IntArgReg4{Registers::a4};
static constexpr Register IntArgReg5{Registers::a5};
static constexpr Register IntArgReg6{Registers::a6};
static constexpr Register IntArgReg7{Registers::a7};
static constexpr Register HeapReg{Registers::s7};

static constexpr Register RegExpTesterRegExpReg{CallTempReg0};
static constexpr Register RegExpTesterStringReg{CallTempReg1};
static constexpr Register RegExpTesterLastIndexReg{CallTempReg2};
static constexpr Register RegExpTesterStickyReg{Registers::invalid_reg};

static constexpr Register RegExpMatcherRegExpReg{CallTempReg0};
static constexpr Register RegExpMatcherStringReg{CallTempReg1};
static constexpr Register RegExpMatcherLastIndexReg{CallTempReg2};
static constexpr Register RegExpMatcherStickyReg{Registers::invalid_reg};

static constexpr Register JSReturnReg_Type{Registers::a3};
static constexpr Register JSReturnReg_Data{Registers::s2};
static constexpr Register JSReturnReg{Registers::a2};

#if defined(JS_NUNBOX32)
static constexpr ValueOperand JSReturnOperand(InvalidReg, InvalidReg);
static constexpr Register64 ReturnReg64(InvalidReg, InvalidReg);
#elif defined(JS_PUNBOX64)
static constexpr ValueOperand JSReturnOperand(InvalidReg);
static constexpr Register64 ReturnReg64(InvalidReg);
#else
#  error "Bad architecture"
#endif

// These registers may be volatile or nonvolatile.
static constexpr Register ABINonArgReg0{Registers::t0};
static constexpr Register ABINonArgReg1{Registers::t1};
static constexpr Register ABINonArgReg2{Registers::t2};
static constexpr Register ABINonArgReg3{Registers::t3};

// These registers may be volatile or nonvolatile.
// Note: these three registers are all guaranteed to be different
static constexpr Register ABINonArgReturnReg0{Registers::t0};
static constexpr Register ABINonArgReturnReg1{Registers::t1};
static constexpr Register ABINonVolatileReg{Registers::s1};

// This register is guaranteed to be clobberable during the prologue and
// epilogue of an ABI call which must preserve both ABI argument, return
// and non-volatile registers.
static constexpr Register ABINonArgReturnVolatileReg{Registers::ra};

static constexpr FloatRegister ABINonArgDoubleReg = {
    FloatRegisters::invalid_reg};

static constexpr Register WasmTableCallScratchReg0{ABINonArgReg0};
static constexpr Register WasmTableCallScratchReg1{ABINonArgReg1};
static constexpr Register WasmTableCallSigReg{ABINonArgReg2};
static constexpr Register WasmTableCallIndexReg{ABINonArgReg3};

// Instance pointer argument register for WebAssembly functions. This must not
// alias any other register used for passing function arguments or return
// values. Preserved by WebAssembly functions. Must be nonvolatile.
static constexpr Register InstanceReg{Registers::s4};

static constexpr Register WasmJitEntryReturnScratch{Registers::t1};

static constexpr Register WasmCallRefCallScratchReg0{ABINonArgReg0};
static constexpr Register WasmCallRefCallScratchReg1{ABINonArgReg1};
static constexpr Register WasmCallRefReg{ABINonArgReg3};

static constexpr uint32_t ABIStackAlignment = 16;
static constexpr uint32_t CodeAlignment = 16;
static constexpr uint32_t JitStackAlignment = 16;
static constexpr uint32_t JitStackValueAlignment =
    JitStackAlignment / sizeof(Value);

static const Scale ScalePointer = TimesEight;

class Assembler : public AssemblerShared {
 public:
  enum Condition {
    Equal,
    NotEqual,
    Above,
    AboveOrEqual,
    Below,
    BelowOrEqual,
    GreaterThan,
    GreaterThanOrEqual,
    LessThan,
    LessThanOrEqual,
    Overflow,
    CarrySet,
    CarryClear,
    Signed,
    NotSigned,
    Zero,
    NonZero,
    Always,
  };

  enum DoubleCondition {
    DoubleOrdered,
    DoubleEqual,
    DoubleNotEqual,
    DoubleGreaterThan,
    DoubleGreaterThanOrEqual,
    DoubleLessThan,
    DoubleLessThanOrEqual,
    DoubleUnordered,
    DoubleEqualOrUnordered,
    DoubleNotEqualOrUnordered,
    DoubleGreaterThanOrUnordered,
    DoubleGreaterThanOrEqualOrUnordered,
    DoubleLessThanOrUnordered,
    DoubleLessThanOrEqualOrUnordered
  };

  static Condition InvertCondition(Condition) { MOZ_CRASH(); }

  static DoubleCondition InvertCondition(DoubleCondition) { MOZ_CRASH(); }

  template <typename T, typename S>
  static void PatchDataWithValueCheck(CodeLocationLabel, T, S) {
    MOZ_CRASH();
  }
  static void PatchWrite_Imm32(CodeLocationLabel, Imm32) { MOZ_CRASH(); }

  static void PatchWrite_NearCall(CodeLocationLabel, CodeLocationLabel) {
    MOZ_CRASH();
  }
  static uint32_t PatchWrite_NearCallSize() { MOZ_CRASH(); }

  static void ToggleToJmp(CodeLocationLabel) { MOZ_CRASH(); }
  static void ToggleToCmp(CodeLocationLabel) { MOZ_CRASH(); }
  static void ToggleCall(CodeLocationLabel, bool) { MOZ_CRASH(); }

  static void Bind(uint8_t*, const CodeLabel&) { MOZ_CRASH(); }

  static uintptr_t GetPointer(uint8_t*) { MOZ_CRASH(); }

  static bool HasRoundInstruction(RoundingMode) { return false; }

  void verifyHeapAccessDisassembly(uint32_t begin, uint32_t end,
                                   const Disassembler::HeapAccess& heapAccess) {
    MOZ_CRASH();
  }

  void setUnlimitedBuffer() { MOZ_CRASH(); }
};

class Operand {
 public:
  explicit Operand(const Address&) { MOZ_CRASH(); }
  explicit Operand(const Register) { MOZ_CRASH(); }
  explicit Operand(const FloatRegister) { MOZ_CRASH(); }
  explicit Operand(Register, Imm32) { MOZ_CRASH(); }
  explicit Operand(Register, int32_t) { MOZ_CRASH(); }
};

class ABIArgGenerator {
 public:
  ABIArgGenerator() { MOZ_CRASH(); }
  ABIArg next(MIRType) { MOZ_CRASH(); }
  ABIArg& current() { MOZ_CRASH(); }
  uint32_t stackBytesConsumedSoFar() const { MOZ_CRASH(); }
  void increaseStackOffset(uint32_t) { MOZ_CRASH(); }
};

// Helper classes for ScratchRegister usage. Asserts that only one piece
// of code thinks it has exclusive ownership of each scratch register.
struct ScratchRegisterScope : public AutoRegisterScope {
  explicit ScratchRegisterScope(MacroAssembler& masm)
      : AutoRegisterScope(masm, ScratchRegister) {}
};

struct SecondScratchRegisterScope : public AutoRegisterScope {
  explicit SecondScratchRegisterScope(MacroAssembler& masm)
      : AutoRegisterScope(masm, SecondScratchReg) {}
};


}  // namespace jit
}  // namespace js

#endif /* jit_riscv64_Assembler_riscv64_h */
