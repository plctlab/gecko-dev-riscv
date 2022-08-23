/* -*- Mode: C++; tab-width: 8; indent-tabs-mode: nil; c-basic-offset: 2 -*-
 * vim: set ts=8 sts=2 et sw=2 tw=80:
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */

#ifndef jit_riscv64_Assembler_riscv64_h
#define jit_riscv64_Assembler_riscv64_h

#include "mozilla/Assertions.h"

#include <stdint.h>

#include "jit/CompactBuffer.h"
#include "jit/JitCode.h"
#include "jit/JitSpewer.h"
#include "jit/Registers.h"
#include "jit/RegisterSets.h"
#include "jit/riscv64/Architecture-riscv64.h"
#include "jit/riscv64/extension/base-assembler-riscv.h"
#include "jit/riscv64/extension/base-riscv-i.h"
#include "jit/riscv64/extension/extension-riscv-a.h"
#include "jit/riscv64/extension/extension-riscv-c.h"
#include "jit/riscv64/extension/extension-riscv-d.h"
#include "jit/riscv64/extension/extension-riscv-f.h"
#include "jit/riscv64/extension/extension-riscv-m.h"
#include "jit/riscv64/extension/extension-riscv-v.h"
#include "jit/riscv64/extension/extension-riscv-zicsr.h"
#include "jit/riscv64/extension/extension-riscv-zifencei.h"
#include "jit/riscv64/Register-riscv64.h"
#include "jit/shared/Assembler-shared.h"
#include "jit/shared/Disassembler-shared.h"
#include "jit/shared/IonAssemblerBuffer.h"
#include "wasm/WasmTypeDecls.h"
namespace js {
namespace jit {

#define DEBUG_PRINTF(...) \
  if (FLAG_riscv_debug) { \
    printf(__VA_ARGS__);  \
  }

// Difference between address of current opcode and value read from pc
// register.
static constexpr int kPcLoadDelta = 4;

// Bits available for offset field in branches
static constexpr int kBranchOffsetBits = 13;

// Bits available for offset field in jump
static constexpr int kJumpOffsetBits = 21;

// Bits available for offset field in compresed jump
static constexpr int kCJalOffsetBits = 12;

// Bits available for offset field in compressed branch
static constexpr int kCBranchOffsetBits = 9;

// Max offset for b instructions with 12-bit offset field (multiple of 2)
static constexpr int kMaxBranchOffset = (1 << (13 - 1)) - 1;

// Max offset for jal instruction with 20-bit offset field (multiple of 2)
static constexpr int kMaxJumpOffset = (1 << (21 - 1)) - 1;

static constexpr int kTrampolineSlotsSize = 2 * kInstrSize;
struct ScratchFloat32Scope : public AutoFloatRegisterScope {
  explicit ScratchFloat32Scope(MacroAssembler& masm)
      : AutoFloatRegisterScope(masm, ScratchFloat32Reg) {}
};

struct ScratchDoubleScope : public AutoFloatRegisterScope {
  explicit ScratchDoubleScope(MacroAssembler& masm)
      : AutoFloatRegisterScope(masm, ScratchDoubleReg) {}
};

class MacroAssembler;


static constexpr uint32_t ABIStackAlignment = 16;
static constexpr uint32_t CodeAlignment = 16;
static constexpr uint32_t JitStackAlignment = 16;
static constexpr uint32_t JitStackValueAlignment =
    JitStackAlignment / sizeof(Value);

static const Scale ScalePointer = TimesEight;

static constexpr int32_t SliceSize = 1024;
typedef js::jit::AssemblerBuffer<SliceSize, Instruction> Buffer;
class Assembler : public AssemblerShared,
                  public AssemblerRISCVI,
                  public AssemblerRISCVA,
                  public AssemblerRISCVF,
                  public AssemblerRISCVD,
                  public AssemblerRISCVM,
                  public AssemblerRISCVC,
                  public AssemblerRISCVZicsr,
                  public AssemblerRISCVZifencei {
 Buffer m_buffer;
 CompactBufferWriter jumpRelocations_;
 CompactBufferWriter dataRelocations_;

#ifdef JS_JITSPEW
  Sprinter* printer;
#endif

 protected:
  bool isFinished;

  Instruction* editSrc(BufferOffset bo) { return m_buffer.getInst(bo); }

 public:
  static bool FLAG_riscv_debug;

  Assembler()
      : m_buffer(),
#ifdef JS_JITSPEW
        printer(nullptr),
#endif
        isFinished(false) {
  }
  bool oom() const;
  BufferOffset nextOffset() { return m_buffer.nextOffset(); }

#ifdef JS_JITSPEW
  inline void spew(const char* fmt, ...) MOZ_FORMAT_PRINTF(2, 3) {
    if (MOZ_UNLIKELY(printer || JitSpewEnabled(JitSpew_Codegen))) {
      va_list va;
      va_start(va, fmt);
      spew(fmt, va);
      va_end(va);
    }
  }

#else
  MOZ_ALWAYS_INLINE void spew(const char* fmt, ...) MOZ_FORMAT_PRINTF(2, 3) {}
#endif

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
    // These conditions will only evaluate to true if the comparison is ordered
    // - i.e. neither operand is NaN.
    DoubleOrdered,
    DoubleEqual,
    DoubleNotEqual,
    DoubleGreaterThan,
    DoubleGreaterThanOrEqual,
    DoubleLessThan,
    DoubleLessThanOrEqual,
    // If either operand is NaN, these conditions always evaluate to true.
    DoubleUnordered,
    DoubleEqualOrUnordered,
    DoubleNotEqualOrUnordered,
    DoubleGreaterThanOrUnordered,
    DoubleGreaterThanOrEqualOrUnordered,
    DoubleLessThanOrUnordered,
    DoubleLessThanOrEqualOrUnordered
  };

  void disassembleInstr(Instr instr);
  int target_at(BufferOffset pos, bool is_internal);
  uint32_t next_link(Label* label, bool is_internal);
  void target_at_put(BufferOffset pos, BufferOffset target_pos);
  virtual int32_t branch_offset_helper(Label* L, OffsetSize bits) { MOZ_CRASH(); }

  virtual void emit(Instr x) { MOZ_CRASH(); }
  virtual void emit(ShortInstr x) { MOZ_CRASH(); }
  virtual void emit(uint64_t x) { MOZ_CRASH(); }

  virtual void BlockTrampolinePoolFor(int instructions) { MOZ_CRASH(); }

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
  // label operations
  void bind(Label* label, BufferOffset boff = BufferOffset());
  void bind(CodeLabel* label) {
    label->target()->bind(currentOffset());
  }
  uint32_t currentOffset() { return nextOffset().getOffset(); }

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
  ABIArgGenerator()
      : intRegIndex_(0), floatRegIndex_(0), stackOffset_(0), current_() {}
  ABIArg next(MIRType);
  ABIArg& current() { return current_; }
  uint32_t stackBytesConsumedSoFar() const { return stackOffset_; }
  void increaseStackOffset(uint32_t bytes) { stackOffset_ += bytes; }
 protected:
  unsigned intRegIndex_;
  unsigned floatRegIndex_;
  uint32_t stackOffset_;
  ABIArg current_;
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

static const uint32_t NumIntArgRegs = 8;
static const uint32_t NumFloatArgRegs = 8;

static inline bool GetIntArgReg(uint32_t usedIntArgs, Register& out) {
  if (usedIntArgs < NumIntArgRegs) {
    out = Register::FromCode(a0.code() + usedIntArgs);
    return true;
  }
  return false;
}

static inline bool GetFloatArgReg(uint32_t usedFloatArgs, FloatRegister* out) {
  if (usedFloatArgs < NumFloatArgRegs) {
    *out = FloatRegister::FromCode(fa0.code() + usedFloatArgs);
    return true;
  }
  return false;
}
}  // namespace jit
}  // namespace js

#endif /* jit_riscv64_Assembler_riscv64_h */
