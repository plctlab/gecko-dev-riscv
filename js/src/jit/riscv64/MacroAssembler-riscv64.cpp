/* -*- Mode: C++; tab-width: 8; indent-tabs-mode: nil; c-basic-offset: 2 -*-
 * vim: set ts=8 sts=2 et sw=2 tw=80:
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */

// Copyright 2021 the V8 project authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.
#include "jit/riscv64/MacroAssembler-riscv64.h"

#include "jsmath.h"

#include "jit/Bailouts.h"
#include "jit/BaselineFrame.h"
#include "jit/JitFrames.h"
#include "jit/JitRuntime.h"
#include "jit/MacroAssembler.h"
#include "jit/MoveEmitter.h"
#include "jit/riscv64/SharedICRegisters-riscv64.h"
#include "util/Memory.h"
#include "vm/JitActivation.h"  // jit::JitActivation
#include "vm/JSContext.h"

#include "jit/MacroAssembler-inl.h"

namespace js {
namespace jit {

MacroAssembler& MacroAssemblerRiscv64::asMasm() {
  return *static_cast<MacroAssembler*>(this);
}

const MacroAssembler& MacroAssemblerRiscv64::asMasm() const {
  return *static_cast<const MacroAssembler*>(this);
}

void MacroAssemblerRiscv64::ma_cmp_set(Register rd,
                                       Register rj,
                                       ImmWord imm,
                                       Condition c) {
  if (imm.value <= INT32_MAX) {
    ma_cmp_set(rd, rj, Imm32(uint32_t(imm.value)), c);
  } else {
    UseScratchRegisterScope temps(this);
    Register scratch = temps.Acquire();
    ma_li(scratch, imm);
    ma_cmp_set(rd, rj, scratch, c);
  }
}

void MacroAssemblerRiscv64::ma_cmp_set(Register rd,
                                       Register rj,
                                       ImmPtr imm,
                                       Condition c) {
  ma_cmp_set(rd, rj, ImmWord(uintptr_t(imm.value)), c);
}

void MacroAssemblerRiscv64::ma_cmp_set(Register rd,
                                       Address address,
                                       Imm32 imm,
                                       Condition c) {
  // TODO(loong64): 32-bit ma_cmp_set?
  UseScratchRegisterScope temps(this);
  Register scratch2 = temps.Acquire();
  ma_load(scratch2, address, SizeWord);
  ma_cmp_set(rd, Register(scratch2), imm, c);
}

void MacroAssemblerRiscv64::ma_cmp_set(Register rd,
                                       Address address,
                                       ImmWord imm,
                                       Condition c) {
  UseScratchRegisterScope temps(this);
  Register scratch2 = temps.Acquire();
  ma_load(scratch2, address, SizeDouble);
  ma_cmp_set(rd, Register(scratch2), imm, c);
}

void MacroAssemblerRiscv64::ma_cmp_set(Register rd,
                                       Register rj,
                                       Imm32 imm,
                                       Condition c) {
  if (imm.value == 0) {
    switch (c) {
      case Equal:
      case BelowOrEqual:
        ma_sltu(rd, rd, Operand(1));
        break;
      case NotEqual:
      case Above:
        sltu(rd, zero, rj);
        break;
      case AboveOrEqual:
      case Below:
        ori(rd, zero, c == AboveOrEqual ? 1 : 0);
        break;
      case GreaterThan:
      case LessThanOrEqual:
        slt(rd, zero, rj);
        if (c == LessThanOrEqual) {
          xori(rd, rd, 1);
        }
        break;
      case LessThan:
      case GreaterThanOrEqual:
        slt(rd, rj, zero);
        if (c == GreaterThanOrEqual) {
          xori(rd, rd, 1);
        }
        break;
      case Zero:
        ma_sltu(rd, rd, Operand(1));
        break;
      case NonZero:
        sltu(rd, zero, rj);
        break;
      case Signed:
        slt(rd, rj, zero);
        break;
      case NotSigned:
        slt(rd, rj, zero);
        xori(rd, rd, 1);
        break;
      default:
        MOZ_CRASH("Invalid condition.");
    }
    return;
  }

  switch (c) {
    case Equal:
    case NotEqual:
      ma_xor(rd, rj, imm);
      if (c == Equal) {
        ma_sltu(rd, rd, Operand(1));
      } else {
        sltu(rd, zero, rd);
      }
      break;
    case Zero:
    case NonZero:
    case Signed:
    case NotSigned:
      MOZ_CRASH("Invalid condition.");
    default:
      Condition cond = ma_cmp(rd, rj, imm, c);
      MOZ_ASSERT(cond == Equal || cond == NotEqual);

      if (cond == Equal)
        xori(rd, rd, 1);
  }
}

Assembler::Condition MacroAssemblerRiscv64::ma_cmp(Register dest,
                                                   Register lhs,
                                                   Register rhs,
                                                   Condition c) {
  switch (c) {
    case Above:
      // bgtu s,t,label =>
      //   sltu at,t,s
      //   bne at,$zero,offs
      sltu(dest, rhs, lhs);
      return NotEqual;
    case AboveOrEqual:
      // bgeu s,t,label =>
      //   sltu at,s,t
      //   beq at,$zero,offs
      sltu(dest, lhs, rhs);
      return Equal;
    case Below:
      // bltu s,t,label =>
      //   sltu at,s,t
      //   bne at,$zero,offs
      sltu(dest, lhs, rhs);
      return NotEqual;
    case BelowOrEqual:
      // bleu s,t,label =>
      //   sltu at,t,s
      //   beq at,$zero,offs
      sltu(dest, rhs, lhs);
      return Equal;
    case GreaterThan:
      // bgt s,t,label =>
      //   slt at,t,s
      //   bne at,$zero,offs
      slt(dest, rhs, lhs);
      return NotEqual;
    case GreaterThanOrEqual:
      // bge s,t,label =>
      //   slt at,s,t
      //   beq at,$zero,offs
      slt(dest, lhs, rhs);
      return Equal;
    case LessThan:
      // blt s,t,label =>
      //   slt at,s,t
      //   bne at,$zero,offs
      slt(dest, lhs, rhs);
      return NotEqual;
    case LessThanOrEqual:
      // ble s,t,label =>
      //   slt at,t,s
      //   beq at,$zero,offs
      slt(dest, rhs, lhs);
      return Equal;
    default:
      MOZ_CRASH("Invalid condition.");
  }
  return Always;
}

Assembler::Condition MacroAssemblerRiscv64::ma_cmp(Register dest,
                                                   Register lhs,
                                                   Imm32 imm,
                                                   Condition c) {
  UseScratchRegisterScope temps(this);
  Register scratch = temps.Acquire();
  MOZ_RELEASE_ASSERT(lhs != scratch);

  switch (c) {
    case Above:
    case BelowOrEqual:
      if (imm.value != 0x7fffffff && is_intn(imm.value + 1, 12) &&
          imm.value != -1) {
        // lhs <= rhs via lhs < rhs + 1 if rhs + 1 does not overflow
        ma_sltu(dest, lhs, Operand(imm.value + 1));

        return (c == BelowOrEqual ? NotEqual : Equal);
      } else {
        ma_li(scratch, imm);
        sltu(dest, scratch, lhs);
        return (c == BelowOrEqual ? Equal : NotEqual);
      }
    case AboveOrEqual:
    case Below:
      if (is_intn(imm.value, 12)) {
        ma_sltu(dest, lhs, Operand(imm.value));
      } else {
        ma_li(scratch, imm);
        sltu(dest, lhs, scratch);
      }
      return (c == AboveOrEqual ? Equal : NotEqual);
    case GreaterThan:
    case LessThanOrEqual:
      if (imm.value != 0x7fffffff && is_intn(imm.value + 1, 12)) {
        // lhs <= rhs via lhs < rhs + 1.
        ma_slt(dest, lhs, Operand(imm.value + 1));
        return (c == LessThanOrEqual ? NotEqual : Equal);
      } else {
        ma_li(scratch, imm);
        slt(dest, scratch, lhs);
        return (c == LessThanOrEqual ? Equal : NotEqual);
      }
    case GreaterThanOrEqual:
    case LessThan:
      if (is_intn(imm.value, 12)) {
        ma_slt(dest, lhs, imm);
      } else {
        ma_li(scratch, imm);
        slt(dest, lhs, scratch);
      }
      return (c == GreaterThanOrEqual ? Equal : NotEqual);
    default:
      MOZ_CRASH("Invalid condition.");
  }
  return Always;
}

void MacroAssemblerRiscv64::ma_cmp_set(Register rd,
                                       Register rj,
                                       Register rk,
                                       Condition c) {
  switch (c) {
    case Equal:
      // seq d,s,t =>
      //   xor d,s,t
      //   sltiu d,d,1
      xor_(rd, rj, rk);
      ma_sltu(rd, rd, Operand(1));
      break;
    case NotEqual:
      // sne d,s,t =>
      //   xor d,s,t
      //   sltu d,$zero,d
      xor_(rd, rj, rk);
      sltu(rd, zero, rd);
      break;
    case Above:
      // sgtu d,s,t =>
      //   sltu d,t,s
      sltu(rd, rk, rj);
      break;
    case AboveOrEqual:
      // sgeu d,s,t =>
      //   sltu d,s,t
      //   xori d,d,1
      sltu(rd, rj, rk);
      xori(rd, rd, 1);
      break;
    case Below:
      // sltu d,s,t
      sltu(rd, rj, rk);
      break;
    case BelowOrEqual:
      // sleu d,s,t =>
      //   sltu d,t,s
      //   xori d,d,1
      sltu(rd, rk, rj);
      xori(rd, rd, 1);
      break;
    case GreaterThan:
      // sgt d,s,t =>
      //   slt d,t,s
      slt(rd, rk, rj);
      break;
    case GreaterThanOrEqual:
      // sge d,s,t =>
      //   slt d,s,t
      //   xori d,d,1
      slt(rd, rj, rk);
      xori(rd, rd, 1);
      break;
    case LessThan:
      // slt d,s,t
      slt(rd, rj, rk);
      break;
    case LessThanOrEqual:
      // sle d,s,t =>
      //   slt d,t,s
      //   xori d,d,1
      slt(rd, rk, rj);
      xori(rd, rd, 1);
      break;
    case Zero:
      MOZ_ASSERT(rj == rk);
      // seq d,s,$zero =>
      //   sltiu d,s,1
      ma_sltu(rd, rj, Operand(1));
      break;
    case NonZero:
      MOZ_ASSERT(rj == rk);
      // sne d,s,$zero =>
      //   sltu d,$zero,s
      sltu(rd, zero, rj);
      break;
    case Signed:
      MOZ_ASSERT(rj == rk);
      slt(rd, rj, zero);
      break;
    case NotSigned:
      MOZ_ASSERT(rj == rk);
      // sge d,s,$zero =>
      //   slt d,s,$zero
      //   xori d,d,1
      slt(rd, rj, zero);
      xori(rd, rd, 1);
      break;
    default:
      MOZ_CRASH("Invalid condition.");
  }
}

void MacroAssemblerRiscv64::ma_compareF32(Register rd,
                                          DoubleCondition cc,
                                          FloatRegister cmp1,
                                          FloatRegister cmp2) {
  switch (cc) {
    case DoubleEqualOrUnordered:
    case DoubleEqual:
      feq_s(rd, cmp1, cmp2);
      break;
    case DoubleNotEqualOrUnordered:
    case DoubleNotEqual:
      feq_s(rd, cmp1, cmp2);
      NegateBool(rd, rd);
      break;
    case DoubleLessThanOrUnordered:
    case DoubleLessThan:
      flt_s(rd, cmp1, cmp2);
      break;
    case DoubleGreaterThanOrEqualOrUnordered:
    case DoubleGreaterThanOrEqual:
      fle_s(rd, cmp2, cmp1);
      break;
    case DoubleLessThanOrEqualOrUnordered:
    case DoubleLessThanOrEqual:
      fle_s(rd, cmp1, cmp2);
      break;
    case DoubleGreaterThanOrUnordered:
    case DoubleGreaterThan:
      flt_s(rd, cmp2, cmp1);
      break;
  }
  if (cc >= FIRST_UNORDERED && cc <= LAST_UNORDERED) {
    UseScratchRegisterScope temps(this);
    Register scratch = temps.Acquire();
    CompareIsNanF32(scratch, cmp1, cmp2);
    or_(rd, rd, scratch);
  }
}

void MacroAssemblerRiscv64::ma_compareF64(Register rd,
                                          DoubleCondition cc,
                                          FloatRegister cmp1,
                                          FloatRegister cmp2) {
  switch (cc) {
    case DoubleEqualOrUnordered:
    case DoubleEqual:
      feq_d(rd, cmp1, cmp2);
      break;
    case DoubleNotEqualOrUnordered:
    case DoubleNotEqual:
      feq_d(rd, cmp1, cmp2);
      NegateBool(rd, rd);
      break;
    case DoubleLessThanOrUnordered:
    case DoubleLessThan:
      flt_d(rd, cmp1, cmp2);
      break;
    case DoubleGreaterThanOrEqualOrUnordered:
    case DoubleGreaterThanOrEqual:
      fle_d(rd, cmp2, cmp1);
      break;
    case DoubleLessThanOrEqualOrUnordered:
    case DoubleLessThanOrEqual:
      fle_d(rd, cmp1, cmp2);
      break;
    case DoubleGreaterThanOrUnordered:
    case DoubleGreaterThan:
      flt_d(rd, cmp2, cmp1);
      break;
  }
  if (cc >= FIRST_UNORDERED && cc <= LAST_UNORDERED) {
    UseScratchRegisterScope temps(this);
    Register scratch = temps.Acquire();
    CompareIsNanF32(scratch, cmp1, cmp2);
    or_(rd, rd, scratch);
  }
}

void MacroAssemblerRiscv64Compat::movePtr(Register src, Register dest) {
  mv(dest, src);
}
void MacroAssemblerRiscv64Compat::movePtr(ImmWord imm, Register dest) {
  ma_li(dest, imm);
}

void MacroAssemblerRiscv64Compat::movePtr(ImmGCPtr imm, Register dest) {
  ma_li(dest, imm);
}

void MacroAssemblerRiscv64Compat::movePtr(ImmPtr imm, Register dest) {
  movePtr(ImmWord(uintptr_t(imm.value)), dest);
}
void MacroAssemblerRiscv64Compat::movePtr(wasm::SymbolicAddress imm,
                                          Register dest) {
  append(wasm::SymbolicAccess(CodeOffset(nextOffset().getOffset()), imm));
  ma_liPatchable(dest, ImmWord(-1));
}

bool MacroAssemblerRiscv64Compat::buildOOLFakeExitFrame(void* fakeReturnAddr) {
  asMasm().PushFrameDescriptor(FrameType::IonJS);  // descriptor_
  asMasm().Push(ImmPtr(fakeReturnAddr));
  asMasm().Push(FramePointer);
  return true;
}

void MacroAssemblerRiscv64Compat::convertUInt32ToDouble(Register src,
                                                        FloatRegister dest) {
  fcvt_d_wu(dest, src);
}

void MacroAssemblerRiscv64Compat::convertUInt64ToDouble(Register src,
                                                        FloatRegister dest) {
  fcvt_d_lu(dest, src);
}

void MacroAssemblerRiscv64Compat::convertUInt32ToFloat32(Register src,
                                                         FloatRegister dest) {
  fcvt_s_wu(dest, src);
}

void MacroAssemblerRiscv64Compat::convertDoubleToFloat32(FloatRegister src,
                                                         FloatRegister dest) {
  fcvt_s_d(dest, src);
}

template <typename F>
void MacroAssemblerRiscv64::RoundHelper(FPURegister dst,
                                        FPURegister src,
                                        FPURegister fpu_scratch,
                                        FPURoundingMode frm) {
  BlockTrampolinePoolScope block_trampoline_pool(this);
  UseScratchRegisterScope temps(this);
  Register scratch2 = temps.Acquire();

  MOZ_ASSERT((std::is_same<float, F>::value) ||
             (std::is_same<double, F>::value));
  // Need at least two FPRs, so check against dst == src == fpu_scratch
  MOZ_ASSERT(!(dst == src && dst == fpu_scratch));

  const int kFloatMantissaBits =
      sizeof(F) == 4 ? kFloat32MantissaBits : kFloat64MantissaBits;
  const int kFloatExponentBits =
      sizeof(F) == 4 ? kFloat32ExponentBits : kFloat64ExponentBits;
  const int kFloatExponentBias =
      sizeof(F) == 4 ? kFloat32ExponentBias : kFloat64ExponentBias;
  Label done;

  {
    UseScratchRegisterScope temps2(this);
    Register scratch = temps2.Acquire();
    // extract exponent value of the source floating-point to scratch
    if (std::is_same<F, double>::value) {
      fmv_x_d(scratch, src);
    } else {
      fmv_x_w(scratch, src);
    }
    ExtractBits(scratch2, scratch, kFloatMantissaBits, kFloatExponentBits);
  }

  // if src is NaN/+-Infinity/+-Zero or if the exponent is larger than # of bits
  // in mantissa, the result is the same as src, so move src to dest  (to avoid
  // generating another branch)
  if (dst != src) {
    if (std::is_same<F, double>::value) {
      fmv_d(dst, src);
    } else {
      fmv_s(dst, src);
    }
  }
  {
    Label not_NaN;
    UseScratchRegisterScope temps2(this);
    Register scratch = temps2.Acquire();
    // According to the wasm spec
    // (https://webassembly.github.io/spec/core/exec/numerics.html#aux-nans)
    // if input is canonical NaN, then output is canonical NaN, and if input is
    // any other NaN, then output is any NaN with most significant bit of
    // payload is 1. In RISC-V, feq_d will set scratch to 0 if src is a NaN. If
    // src is not a NaN, branch to the label and do nothing, but if it is,
    // fmin_d will set dst to the canonical NaN.
    if (std::is_same<F, double>::value) {
      feq_d(scratch, src, src);
      bnez(scratch, &not_NaN);
      fmin_d(dst, src, src);
    } else {
      feq_s(scratch, src, src);
      bnez(scratch, &not_NaN);
      fmin_s(dst, src, src);
    }
    bind(&not_NaN);
  }

  // If real exponent (i.e., scratch2 - kFloatExponentBias) is greater than
  // kFloat32MantissaBits, it means the floating-point value has no fractional
  // part, thus the input is already rounded, jump to done. Note that, NaN and
  // Infinity in floating-point representation sets maximal exponent value, so
  // they also satisfy (scratch2 - kFloatExponentBias >= kFloatMantissaBits),
  // and JS round semantics specify that rounding of NaN (Infinity) returns NaN
  // (Infinity), so NaN and Infinity are considered rounded value too.
  ma_branch(&done, GreaterThanOrEqual, scratch2,
            Operand(kFloatExponentBias + kFloatMantissaBits));

  // Actual rounding is needed along this path

  // old_src holds the original input, needed for the case of src == dst
  FPURegister old_src = src;
  if (src == dst) {
    MOZ_ASSERT(fpu_scratch != dst);
    fmv_d(fpu_scratch, src);
    old_src = fpu_scratch;
  }

  // Since only input whose real exponent value is less than kMantissaBits
  // (i.e., 23 or 52-bits) falls into this path, the value range of the input
  // falls into that of 23- or 53-bit integers. So we round the input to integer
  // values, then convert them back to floating-point.
  {
    UseScratchRegisterScope temps(this);
    Register scratch = temps.Acquire();
    if (std::is_same<F, double>::value) {
      fcvt_l_d(scratch, src, frm);
      fcvt_d_l(dst, scratch, frm);
    } else {
      fcvt_w_s(scratch, src, frm);
      fcvt_s_w(dst, scratch, frm);
    }
  }
  // A special handling is needed if the input is a very small positive/negative
  // number that rounds to zero. JS semantics requires that the rounded result
  // retains the sign of the input, so a very small positive (negative)
  // floating-point number should be rounded to positive (negative) 0.
  // Therefore, we use sign-bit injection to produce +/-0 correctly. Instead of
  // testing for zero w/ a branch, we just insert sign-bit for everyone on this
  // path (this is where old_src is needed)
  if (std::is_same<F, double>::value) {
    fsgnj_d(dst, dst, old_src);
  } else {
    fsgnj_s(dst, dst, old_src);
  }

  bind(&done);
}

template <typename CvtFunc>
void MacroAssemblerRiscv64::RoundFloatingPointToInteger(
    Register rd,
    FPURegister fs,
    Register result,
    CvtFunc fcvt_generator) {
  // Save csr_fflags to scratch & clear exception flags
  if (result != Register::Invalid()) {
    BlockTrampolinePoolScope block_trampoline_pool(this);
    UseScratchRegisterScope temps(this);
    Register scratch = temps.Acquire();

    int exception_flags = kInvalidOperation;
    csrrci(scratch, csr_fflags, exception_flags);

    // actual conversion instruction
    fcvt_generator(this, rd, fs);

    // check kInvalidOperation flag (out-of-range, NaN)
    // set result to 1 if normal, otherwise set result to 0 for abnormal
    frflags(result);
    andi(result, result, exception_flags);
    seqz(result, result);  // result <-- 1 (normal), result <-- 0 (abnormal)

    // restore csr_fflags
    csrw(csr_fflags, scratch);
  } else {
    // actual conversion instruction
    fcvt_generator(this, rd, fs);
  }
}

void MacroAssemblerRiscv64::Trunc_uw_d(Register rd,
                                       FPURegister fs,
                                       Register result) {
  RoundFloatingPointToInteger(
      rd, fs, result,
      [](MacroAssemblerRiscv64* tasm, Register dst, FPURegister src) {
        tasm->fcvt_wu_d(dst, src, RTZ);
      });
}

void MacroAssemblerRiscv64::Trunc_w_d(Register rd,
                                      FPURegister fs,
                                      Register result) {
  RoundFloatingPointToInteger(
      rd, fs, result,
      [](MacroAssemblerRiscv64* tasm, Register dst, FPURegister src) {
        tasm->fcvt_w_d(dst, src, RTZ);
      });
}

void MacroAssemblerRiscv64::Trunc_uw_s(Register rd,
                                       FPURegister fs,
                                       Register result) {
  RoundFloatingPointToInteger(
      rd, fs, result,
      [](MacroAssemblerRiscv64* tasm, Register dst, FPURegister src) {
        tasm->fcvt_wu_s(dst, src, RTZ);
      });
}

void MacroAssemblerRiscv64::Trunc_w_s(Register rd,
                                      FPURegister fs,
                                      Register result) {
  RoundFloatingPointToInteger(
      rd, fs, result,
      [](MacroAssemblerRiscv64* tasm, Register dst, FPURegister src) {
        tasm->fcvt_w_s(dst, src, RTZ);
      });
}
void MacroAssemblerRiscv64::Trunc_ul_d(Register rd,
                                       FPURegister fs,
                                       Register result) {
  RoundFloatingPointToInteger(
      rd, fs, result,
      [](MacroAssemblerRiscv64* tasm, Register dst, FPURegister src) {
        tasm->fcvt_lu_d(dst, src, RTZ);
      });
}

void MacroAssemblerRiscv64::Trunc_l_d(Register rd,
                                      FPURegister fs,
                                      Register result) {
  RoundFloatingPointToInteger(
      rd, fs, result,
      [](MacroAssemblerRiscv64* tasm, Register dst, FPURegister src) {
        tasm->fcvt_l_d(dst, src, RTZ);
      });
}

void MacroAssemblerRiscv64::Trunc_ul_s(Register rd,
                                       FPURegister fs,
                                       Register result) {
  RoundFloatingPointToInteger(
      rd, fs, result,
      [](MacroAssemblerRiscv64* tasm, Register dst, FPURegister src) {
        tasm->fcvt_lu_s(dst, src, RTZ);
      });
}

void MacroAssemblerRiscv64::Trunc_l_s(Register rd,
                                      FPURegister fs,
                                      Register result) {
  RoundFloatingPointToInteger(
      rd, fs, result,
      [](MacroAssemblerRiscv64* tasm, Register dst, FPURegister src) {
        tasm->fcvt_l_s(dst, src, RTZ);
      });
}

void MacroAssemblerRiscv64::Floor_d_d(FPURegister dst,
                                      FPURegister src,
                                      FPURegister fpu_scratch) {
  RoundHelper<double>(dst, src, fpu_scratch, RDN);
}

void MacroAssemblerRiscv64::Ceil_d_d(FPURegister dst,
                                     FPURegister src,
                                     FPURegister fpu_scratch) {
  RoundHelper<double>(dst, src, fpu_scratch, RUP);
}

void MacroAssemblerRiscv64::Trunc_d_d(FPURegister dst,
                                      FPURegister src,
                                      FPURegister fpu_scratch) {
  RoundHelper<double>(dst, src, fpu_scratch, RTZ);
}

void MacroAssemblerRiscv64::Round_d_d(FPURegister dst,
                                      FPURegister src,
                                      FPURegister fpu_scratch) {
  RoundHelper<double>(dst, src, fpu_scratch, RNE);
}

void MacroAssemblerRiscv64::Floor_s_s(FPURegister dst,
                                      FPURegister src,
                                      FPURegister fpu_scratch) {
  RoundHelper<float>(dst, src, fpu_scratch, RDN);
}

void MacroAssemblerRiscv64::Ceil_s_s(FPURegister dst,
                                     FPURegister src,
                                     FPURegister fpu_scratch) {
  RoundHelper<float>(dst, src, fpu_scratch, RUP);
}

void MacroAssemblerRiscv64::Trunc_s_s(FPURegister dst,
                                      FPURegister src,
                                      FPURegister fpu_scratch) {
  RoundHelper<float>(dst, src, fpu_scratch, RTZ);
}

void MacroAssemblerRiscv64::Round_s_s(FPURegister dst,
                                      FPURegister src,
                                      FPURegister fpu_scratch) {
  RoundHelper<float>(dst, src, fpu_scratch, RNE);
}

void MacroAssemblerRiscv64::Round_w_s(Register rd,
                                      FPURegister fs,
                                      Register result) {
  RoundFloatingPointToInteger(
      rd, fs, result,
      [](MacroAssemblerRiscv64* tasm, Register dst, FPURegister src) {
        tasm->fcvt_w_s(dst, src, RNE);
      });
}

void MacroAssemblerRiscv64::Round_w_d(Register rd,
                                      FPURegister fs,
                                      Register result) {
  RoundFloatingPointToInteger(
      rd, fs, result,
      [](MacroAssemblerRiscv64* tasm, Register dst, FPURegister src) {
        tasm->fcvt_w_d(dst, src, RNE);
      });
}

void MacroAssemblerRiscv64::Ceil_w_s(Register rd,
                                     FPURegister fs,
                                     Register result) {
  RoundFloatingPointToInteger(
      rd, fs, result,
      [](MacroAssemblerRiscv64* tasm, Register dst, FPURegister src) {
        tasm->fcvt_w_s(dst, src, RUP);
      });
}

void MacroAssemblerRiscv64::Ceil_w_d(Register rd,
                                     FPURegister fs,
                                     Register result) {
  RoundFloatingPointToInteger(
      rd, fs, result,
      [](MacroAssemblerRiscv64* tasm, Register dst, FPURegister src) {
        tasm->fcvt_w_d(dst, src, RUP);
      });
}

void MacroAssemblerRiscv64::Floor_w_s(Register rd,
                                      FPURegister fs,
                                      Register result) {
  RoundFloatingPointToInteger(
      rd, fs, result,
      [](MacroAssemblerRiscv64* tasm, Register dst, FPURegister src) {
        tasm->fcvt_w_s(dst, src, RDN);
      });
}

void MacroAssemblerRiscv64::Floor_w_d(Register rd,
                                      FPURegister fs,
                                      Register result) {
  RoundFloatingPointToInteger(
      rd, fs, result,
      [](MacroAssemblerRiscv64* tasm, Register dst, FPURegister src) {
        tasm->fcvt_w_d(dst, src, RDN);
      });
}

// Checks whether a double is representable as a 32-bit integer. If so, the
// integer is written to the output register. Otherwise, a bailout is taken to
// the given snapshot. This function overwrites the scratch float register.
void MacroAssemblerRiscv64Compat::convertDoubleToInt32(FloatRegister src,
                                                       Register dest,
                                                       Label* fail,
                                                       bool negativeZeroCheck) {
  if (negativeZeroCheck) {
    fclass_d(dest, src);
    ma_b(dest, Imm32(kNegativeZero), fail, Equal);
  }
  UseScratchRegisterScope temps(this);
  Register scratch = temps.Acquire();
  Trunc_w_d(dest, src, scratch);
  ma_b(scratch, Imm32(0), fail, Equal);
}

void MacroAssemblerRiscv64Compat::convertDoubleToPtr(FloatRegister src,
                                                     Register dest,
                                                     Label* fail,
                                                     bool negativeZeroCheck) {
  if (negativeZeroCheck) {
    fclass_d(dest, src);
    ma_b(dest, Imm32(kNegativeZero), fail, Equal);
  }
  UseScratchRegisterScope temps(this);
  Register scratch = temps.Acquire();
  Trunc_l_d(dest, src, scratch);
  ma_b(scratch, Imm32(0), fail, Equal);
}

// Checks whether a float32 is representable as a 32-bit integer. If so, the
// integer is written to the output register. Otherwise, a bailout is taken to
// the given snapshot. This function overwrites the scratch float register.
void MacroAssemblerRiscv64Compat::convertFloat32ToInt32(
    FloatRegister src,
    Register dest,
    Label* fail,
    bool negativeZeroCheck) {
  if (negativeZeroCheck) {
    fclass_d(dest, src);
    ma_b(dest, Imm32(kNegativeZero), fail, Equal);
  }
  UseScratchRegisterScope temps(this);
  Register scratch = temps.Acquire();
  Trunc_w_s(dest, src, scratch);
  ma_b(scratch, Imm32(0), fail, Equal);
}

void MacroAssemblerRiscv64Compat::convertFloat32ToDouble(FloatRegister src,
                                                         FloatRegister dest) {
  fcvt_d_s(dest, src);
}

void MacroAssemblerRiscv64Compat::convertInt32ToFloat32(Register src,
                                                        FloatRegister dest) {
  fcvt_s_w(dest, src);
}

void MacroAssemblerRiscv64Compat::convertInt32ToFloat32(const Address& src,
                                                        FloatRegister dest) {
  UseScratchRegisterScope temps(this);
  Register scratch = temps.Acquire();
  load32(src, scratch);
  fcvt_s_w(dest, scratch);
}

void MacroAssemblerRiscv64Compat::movq(Register rj, Register rd) {
  mv(rd, rj);
}

// Memory.
void MacroAssemblerRiscv64::ma_loadDouble(FloatRegister dest, Address address) {
  int16_t encodedOffset;
  Register base;

  if (!is_int12(address.offset)) {
    UseScratchRegisterScope temps(this);
    Register ScratchRegister = temps.Acquire();
    ma_li(ScratchRegister, Imm32(address.offset));
    add(ScratchRegister, address.base, ScratchRegister);
    base = ScratchRegister;
    encodedOffset = 0;
  } else {
    encodedOffset = address.offset;
    base = address.base;
  }
  fld(dest, base, encodedOffset);
}

void MacroAssemblerRiscv64::ma_loadFloat(FloatRegister dest, Address address) {
  int16_t encodedOffset;
  Register base;

  if (!is_int12(address.offset)) {
    UseScratchRegisterScope temps(this);
    Register ScratchRegister = temps.Acquire();
    ma_li(ScratchRegister, Imm32(address.offset));
    add(ScratchRegister, address.base, ScratchRegister);
    base = ScratchRegister;
    encodedOffset = 0;
  } else {
    encodedOffset = address.offset;
    base = address.base;
  }
  flw(dest, base, encodedOffset);
}

void MacroAssemblerRiscv64::ma_load(Register dest,
                                    Address address,
                                    LoadStoreSize size,
                                    LoadStoreExtension extension) {
  int16_t encodedOffset;
  Register base;

  if (!is_int12(address.offset)) {
    UseScratchRegisterScope temps(this);
    Register ScratchRegister = temps.Acquire();
    ma_li(ScratchRegister, Imm32(address.offset));
    add(ScratchRegister, address.base, ScratchRegister);
    base = ScratchRegister;
    encodedOffset = 0;
  } else {
    encodedOffset = address.offset;
    base = address.base;
  }

  switch (size) {
    case SizeByte:
      if (ZeroExtend == extension) {
        lbu(dest, base, encodedOffset);
      } else {
        lb(dest, base, encodedOffset);
      }
      break;
    case SizeHalfWord:
      if (ZeroExtend == extension) {
        lhu(dest, base, encodedOffset);
      } else {
        lh(dest, base, encodedOffset);
      }
      break;
    case SizeWord:
      if (ZeroExtend == extension) {
        lwu(dest, base, encodedOffset);
      } else {
        lw(dest, base, encodedOffset);
      }
      break;
    case SizeDouble:
      ld(dest, base, encodedOffset);
      break;
    default:
      MOZ_CRASH("Invalid argument for ma_load");
  }
}

void MacroAssemblerRiscv64::ma_store(Register data,
                                     const BaseIndex& dest,
                                     LoadStoreSize size,
                                     LoadStoreExtension extension) {
  UseScratchRegisterScope temps(this);
  Register address = temps.Acquire();
  // Make sure that scratch contains absolute address so that
  // offset is 0.
  computeScaledAddress(dest, address);
  // with offset=0 ScratchRegister will not be used in ma_store()
  // so we can use it as a parameter here
  ma_store(data, Address(address, 0), size, extension);
}

void MacroAssemblerRiscv64::ma_store(Imm32 imm,
                                     const BaseIndex& dest,
                                     LoadStoreSize size,
                                     LoadStoreExtension extension) {
  UseScratchRegisterScope temps(this);
  Register scratch = temps.Acquire();
  Register address = temps.Acquire();
  // Make sure that scratch contains absolute address so that
  // offset is 0.
  computeScaledAddress(dest, address);

  // Scrach register is free now, use it for loading imm value
  ma_li(scratch, imm);

  // with offset=0 ScratchRegister will not be used in ma_store()
  // so we can use it as a parameter here
  ma_store(scratch, Address(address, 0), size, extension);
}

void MacroAssemblerRiscv64::ma_store(Imm32 imm,
                                     Address address,
                                     LoadStoreSize size,
                                     LoadStoreExtension extension) {
  UseScratchRegisterScope temps(this);
  Register scratch = temps.Acquire();
  ma_li(scratch, imm);
  ma_store(scratch, address, size, extension);
}

void MacroAssemblerRiscv64::ma_store(Register data,
                                     Address address,
                                     LoadStoreSize size,
                                     LoadStoreExtension extension) {
  int16_t encodedOffset;
  Register base;

  if (!is_int12(address.offset)) {
    UseScratchRegisterScope temps(this);
    Register ScratchRegister = temps.Acquire();
    ma_li(ScratchRegister, Imm32(address.offset));
    add(ScratchRegister, address.base, ScratchRegister);
    base = ScratchRegister;
    encodedOffset = 0;
  } else {
    encodedOffset = address.offset;
    base = address.base;
  }

  switch (size) {
    case SizeByte:
      sb(data, base, encodedOffset);
      break;
    case SizeHalfWord:
      sh(data, base, encodedOffset);
      break;
    case SizeWord:
      sw(data, base, encodedOffset);
      break;
    case SizeDouble:
      sd(data, base, encodedOffset);
      break;
    default:
      MOZ_CRASH("Invalid argument for ma_store");
  }
}

void MacroAssemblerRiscv64::computeScaledAddress(const BaseIndex& address,
                                                 Register dest) {
  Register base = address.base;
  Register index = address.index;
  int32_t shift = Imm32::ShiftOf(address.scale).value;
  UseScratchRegisterScope temps(this);
  Register tmp = dest == base ? temps.Acquire() : dest;
  if (shift) {
    MOZ_ASSERT(shift <= 4);
    slli(tmp, index, shift);
    add(dest, base, tmp);
  } else {
    add(dest, base, index);
  }
}

void MacroAssemblerRiscv64Compat::wasmLoadI64Impl(
    const wasm::MemoryAccessDesc& access,
    Register memoryBase,
    Register ptr,
    Register ptrScratch,
    Register64 output,
    Register tmp) {
  uint32_t offset = access.offset();
  MOZ_ASSERT(offset < asMasm().wasmMaxOffsetGuardLimit());
  MOZ_ASSERT_IF(offset, ptrScratch != InvalidReg);

  MOZ_CRASH("Unimplement riscv");
}

void MacroAssemblerRiscv64Compat::wasmStoreI64Impl(
    const wasm::MemoryAccessDesc& access,
    Register64 value,
    Register memoryBase,
    Register ptr,
    Register ptrScratch,
    Register tmp) {
  uint32_t offset = access.offset();
  MOZ_ASSERT(offset < asMasm().wasmMaxOffsetGuardLimit());
  MOZ_ASSERT_IF(offset, ptrScratch != InvalidReg);

  MOZ_CRASH("Unimplement riscv");
}

void MacroAssemblerRiscv64Compat::profilerEnterFrame(Register framePtr,
                                                     Register scratch) {
  asMasm().loadJSContext(scratch);
  loadPtr(Address(scratch, offsetof(JSContext, profilingActivation_)), scratch);
  storePtr(framePtr,
           Address(scratch, JitActivation::offsetOfLastProfilingFrame()));
  storePtr(ImmPtr(nullptr),
           Address(scratch, JitActivation::offsetOfLastProfilingCallSite()));
}

void MacroAssemblerRiscv64Compat::profilerExitFrame() {
  jump(asMasm().runtime()->jitRuntime()->getProfilerExitFrameTail());
}

void MacroAssemblerRiscv64Compat::move32(Imm32 imm, Register dest) {
  ma_li(dest, imm);
}

void MacroAssemblerRiscv64Compat::move32(Register src, Register dest) {
  slliw(dest, src, 0);
}

void MacroAssemblerRiscv64Compat::load8ZeroExtend(const Address& address,
                                                  Register dest) {
  ma_load(dest, address, SizeByte, ZeroExtend);
}

void MacroAssemblerRiscv64Compat::load8ZeroExtend(const BaseIndex& src,
                                                  Register dest) {
  ma_load(dest, src, SizeByte, ZeroExtend);
}

void MacroAssemblerRiscv64Compat::load8SignExtend(const Address& address,
                                                  Register dest) {
  ma_load(dest, address, SizeByte, SignExtend);
}

void MacroAssemblerRiscv64Compat::load8SignExtend(const BaseIndex& src,
                                                  Register dest) {
  ma_load(dest, src, SizeByte, SignExtend);
}

void MacroAssemblerRiscv64Compat::load16ZeroExtend(const Address& address,
                                                   Register dest) {
  ma_load(dest, address, SizeHalfWord, ZeroExtend);
}

void MacroAssemblerRiscv64Compat::load16ZeroExtend(const BaseIndex& src,
                                                   Register dest) {
  ma_load(dest, src, SizeHalfWord, ZeroExtend);
}

void MacroAssemblerRiscv64Compat::load16SignExtend(const Address& address,
                                                   Register dest) {
  ma_load(dest, address, SizeHalfWord, SignExtend);
}

void MacroAssemblerRiscv64Compat::load16SignExtend(const BaseIndex& src,
                                                   Register dest) {
  ma_load(dest, src, SizeHalfWord, SignExtend);
}

void MacroAssemblerRiscv64Compat::load32(const Address& address,
                                         Register dest) {
  ma_load(dest, address, SizeWord);
}

void MacroAssemblerRiscv64Compat::load32(const BaseIndex& address,
                                         Register dest) {
  ma_load(dest, address, SizeWord);
}

void MacroAssemblerRiscv64Compat::load32(AbsoluteAddress address,
                                         Register dest) {
  UseScratchRegisterScope temps(this);
  Register ScratchRegister = temps.Acquire();
  movePtr(ImmPtr(address.addr), ScratchRegister);
  load32(Address(ScratchRegister, 0), dest);
}

void MacroAssemblerRiscv64Compat::load32(wasm::SymbolicAddress address,
                                         Register dest) {
  UseScratchRegisterScope temps(this);
  Register ScratchRegister = temps.Acquire();
  movePtr(address, ScratchRegister);
  load32(Address(ScratchRegister, 0), dest);
}

void MacroAssemblerRiscv64Compat::loadPtr(const Address& address,
                                          Register dest) {
  ma_load(dest, address, SizeDouble);
}

void MacroAssemblerRiscv64Compat::loadPtr(const BaseIndex& src, Register dest) {
  ma_load(dest, src, SizeDouble);
}

void MacroAssemblerRiscv64Compat::loadPtr(AbsoluteAddress address,
                                          Register dest) {
  UseScratchRegisterScope temps(this);
  Register ScratchRegister = temps.Acquire();
  movePtr(ImmPtr(address.addr), ScratchRegister);
  loadPtr(Address(ScratchRegister, 0), dest);
}

void MacroAssemblerRiscv64Compat::loadPtr(wasm::SymbolicAddress address,
                                          Register dest) {
  UseScratchRegisterScope temps(this);
  Register ScratchRegister = temps.Acquire();
  movePtr(address, ScratchRegister);
  loadPtr(Address(ScratchRegister, 0), dest);
}

void MacroAssemblerRiscv64Compat::loadPrivate(const Address& address,
                                              Register dest) {
  loadPtr(address, dest);
}

void MacroAssemblerRiscv64Compat::store8(Imm32 imm, const Address& address) {
  UseScratchRegisterScope temps(this);
  Register ScratchRegister = temps.Acquire();
  ma_li(ScratchRegister, imm);
  ma_store(ScratchRegister, address, SizeByte);
}

void MacroAssemblerRiscv64Compat::store8(Register src, const Address& address) {
  ma_store(src, address, SizeByte);
}

void MacroAssemblerRiscv64Compat::store8(Imm32 imm, const BaseIndex& dest) {
  ma_store(imm, dest, SizeByte);
}

void MacroAssemblerRiscv64Compat::store8(Register src, const BaseIndex& dest) {
  ma_store(src, dest, SizeByte);
}

void MacroAssemblerRiscv64Compat::store16(Imm32 imm, const Address& address) {
  UseScratchRegisterScope temps(this);
  Register ScratchRegister = temps.Acquire();
  ma_li(ScratchRegister, imm);
  ma_store(ScratchRegister, address, SizeHalfWord);
}

void MacroAssemblerRiscv64Compat::store16(Register src,
                                          const Address& address) {
  ma_store(src, address, SizeHalfWord);
}

void MacroAssemblerRiscv64Compat::store16(Imm32 imm, const BaseIndex& dest) {
  ma_store(imm, dest, SizeHalfWord);
}

void MacroAssemblerRiscv64Compat::store16(Register src,
                                          const BaseIndex& address) {
  ma_store(src, address, SizeHalfWord);
}

void MacroAssemblerRiscv64Compat::store32(Register src,
                                          AbsoluteAddress address) {
  UseScratchRegisterScope temps(this);
  Register ScratchRegister = temps.Acquire();
  movePtr(ImmPtr(address.addr), ScratchRegister);
  store32(src, Address(ScratchRegister, 0));
}

void MacroAssemblerRiscv64Compat::store32(Register src,
                                          const Address& address) {
  ma_store(src, address, SizeWord);
}

void MacroAssemblerRiscv64Compat::store32(Imm32 src, const Address& address) {
  UseScratchRegisterScope temps(this);
  Register ScratchRegister = temps.Acquire();
  move32(src, ScratchRegister);
  ma_store(ScratchRegister, address, SizeWord);
}

void MacroAssemblerRiscv64Compat::store32(Imm32 imm, const BaseIndex& dest) {
  ma_store(imm, dest, SizeWord);
}

void MacroAssemblerRiscv64Compat::store32(Register src, const BaseIndex& dest) {
  ma_store(src, dest, SizeWord);
}

template <typename T>
void MacroAssemblerRiscv64Compat::storePtr(ImmWord imm, T address) {
  UseScratchRegisterScope temps(this);
  Register ScratchRegister = temps.Acquire();
  ma_li(ScratchRegister, imm);
  ma_store(ScratchRegister, address, SizeDouble);
}

template void MacroAssemblerRiscv64Compat::storePtr<Address>(ImmWord imm,
                                                             Address address);
template void MacroAssemblerRiscv64Compat::storePtr<BaseIndex>(
    ImmWord imm,
    BaseIndex address);

template <typename T>
void MacroAssemblerRiscv64Compat::storePtr(ImmPtr imm, T address) {
  storePtr(ImmWord(uintptr_t(imm.value)), address);
}

template void MacroAssemblerRiscv64Compat::storePtr<Address>(ImmPtr imm,
                                                             Address address);
template void MacroAssemblerRiscv64Compat::storePtr<BaseIndex>(
    ImmPtr imm,
    BaseIndex address);

template <typename T>
void MacroAssemblerRiscv64Compat::storePtr(ImmGCPtr imm, T address) {
  UseScratchRegisterScope temps(this);
  Register ScratchRegister = temps.Acquire();
  movePtr(imm, ScratchRegister);
  storePtr(ScratchRegister, address);
}

template void MacroAssemblerRiscv64Compat::storePtr<Address>(ImmGCPtr imm,
                                                             Address address);
template void MacroAssemblerRiscv64Compat::storePtr<BaseIndex>(
    ImmGCPtr imm,
    BaseIndex address);

void MacroAssemblerRiscv64Compat::storePtr(Register src,
                                           const Address& address) {
  ma_store(src, address, SizeDouble);
}

void MacroAssemblerRiscv64Compat::storePtr(Register src,
                                           const BaseIndex& address) {
  ma_store(src, address, SizeDouble);
}

void MacroAssemblerRiscv64Compat::storePtr(Register src, AbsoluteAddress dest) {
  UseScratchRegisterScope temps(this);
  Register ScratchRegister = temps.Acquire();
  movePtr(ImmPtr(dest.addr), ScratchRegister);
  storePtr(src, Address(ScratchRegister, 0));
}

void MacroAssemblerRiscv64Compat::testNullSet(Condition cond,
                                              const ValueOperand& value,
                                              Register dest) {
  MOZ_ASSERT(cond == Equal || cond == NotEqual);
  UseScratchRegisterScope temps(this);
  Register ScratchRegister = temps.Acquire();
  splitTag(value, ScratchRegister);
  ma_cmp_set(dest, ScratchRegister, ImmTag(JSVAL_TAG_NULL), cond);
}

void MacroAssemblerRiscv64Compat::testObjectSet(Condition cond,
                                                const ValueOperand& value,
                                                Register dest) {
  MOZ_ASSERT(cond == Equal || cond == NotEqual);
  UseScratchRegisterScope temps(this);
  Register ScratchRegister = temps.Acquire();
  splitTag(value, ScratchRegister);
  ma_cmp_set(dest, ScratchRegister, ImmTag(JSVAL_TAG_OBJECT), cond);
}

void MacroAssemblerRiscv64Compat::testUndefinedSet(Condition cond,
                                                   const ValueOperand& value,
                                                   Register dest) {
  MOZ_ASSERT(cond == Equal || cond == NotEqual);
  UseScratchRegisterScope temps(this);
  Register ScratchRegister = temps.Acquire();
  splitTag(value, ScratchRegister);
  ma_cmp_set(dest, ScratchRegister, ImmTag(JSVAL_TAG_UNDEFINED), cond);
}

void MacroAssemblerRiscv64Compat::unboxInt32(const ValueOperand& operand,
                                             Register dest) {
  slli(dest, operand.valueReg(), 0);
}

void MacroAssemblerRiscv64Compat::unboxInt32(Register src, Register dest) {
  slli(dest, src, 0);
}

void MacroAssemblerRiscv64Compat::unboxInt32(const Address& src,
                                             Register dest) {
  load32(Address(src.base, src.offset), dest);
}

void MacroAssemblerRiscv64Compat::unboxInt32(const BaseIndex& src,
                                             Register dest) {
  UseScratchRegisterScope temps(this);
  Register ScratchRegister = temps.Acquire();
  computeScaledAddress(src, ScratchRegister);
  load32(Address(ScratchRegister, src.offset), dest);
}

void MacroAssemblerRiscv64Compat::unboxBoolean(const ValueOperand& operand,
                                               Register dest) {
  ExtractBits(dest, operand.valueReg(), 0, 32);
}

void MacroAssemblerRiscv64Compat::unboxBoolean(Register src, Register dest) {
  ExtractBits(dest, src, 0, 32);
}

void MacroAssemblerRiscv64Compat::unboxBoolean(const Address& src,
                                               Register dest) {
  ma_load(dest, Address(src.base, src.offset), SizeWord, ZeroExtend);
}

void MacroAssemblerRiscv64Compat::unboxBoolean(const BaseIndex& src,
                                               Register dest) {
  UseScratchRegisterScope temps(this);
  Register ScratchRegister = temps.Acquire();
  computeScaledAddress(src, ScratchRegister);
  ma_load(dest, Address(ScratchRegister, src.offset), SizeWord, ZeroExtend);
}

void MacroAssemblerRiscv64Compat::unboxDouble(const ValueOperand& operand,
                                              FloatRegister dest) {
  fmv_d_x(dest, operand.valueReg());
}

void MacroAssemblerRiscv64Compat::unboxDouble(const Address& src,
                                              FloatRegister dest) {
  ma_loadDouble(dest, Address(src.base, src.offset));
}

void MacroAssemblerRiscv64Compat::unboxDouble(const BaseIndex& src,
                                              FloatRegister dest) {
  UseScratchRegisterScope temps(this);
  Register scratch = temps.Acquire();
  loadPtr(src, scratch);
  unboxDouble(ValueOperand(scratch), dest);
}

void MacroAssemblerRiscv64Compat::unboxString(const ValueOperand& operand,
                                              Register dest) {
  unboxNonDouble(operand, dest, JSVAL_TYPE_STRING);
}

void MacroAssemblerRiscv64Compat::unboxString(Register src, Register dest) {
  unboxNonDouble(src, dest, JSVAL_TYPE_STRING);
}

void MacroAssemblerRiscv64Compat::unboxString(const Address& src,
                                              Register dest) {
  unboxNonDouble(src, dest, JSVAL_TYPE_STRING);
}

void MacroAssemblerRiscv64Compat::unboxSymbol(const ValueOperand& operand,
                                              Register dest) {
  unboxNonDouble(operand, dest, JSVAL_TYPE_SYMBOL);
}

void MacroAssemblerRiscv64Compat::unboxSymbol(Register src, Register dest) {
  unboxNonDouble(src, dest, JSVAL_TYPE_SYMBOL);
}

void MacroAssemblerRiscv64Compat::unboxSymbol(const Address& src,
                                              Register dest) {
  unboxNonDouble(src, dest, JSVAL_TYPE_SYMBOL);
}

void MacroAssemblerRiscv64Compat::unboxBigInt(const ValueOperand& operand,
                                              Register dest) {
  unboxNonDouble(operand, dest, JSVAL_TYPE_BIGINT);
}

void MacroAssemblerRiscv64Compat::unboxBigInt(Register src, Register dest) {
  unboxNonDouble(src, dest, JSVAL_TYPE_BIGINT);
}

void MacroAssemblerRiscv64Compat::unboxBigInt(const Address& src,
                                              Register dest) {
  unboxNonDouble(src, dest, JSVAL_TYPE_BIGINT);
}

void MacroAssemblerRiscv64Compat::unboxObject(const ValueOperand& src,
                                              Register dest) {
  unboxNonDouble(src, dest, JSVAL_TYPE_OBJECT);
}

void MacroAssemblerRiscv64Compat::unboxObject(Register src, Register dest) {
  unboxNonDouble(src, dest, JSVAL_TYPE_OBJECT);
}

void MacroAssemblerRiscv64Compat::unboxObject(const Address& src,
                                              Register dest) {
  unboxNonDouble(src, dest, JSVAL_TYPE_OBJECT);
}

void MacroAssemblerRiscv64Compat::unboxValue(const ValueOperand& src,
                                             AnyRegister dest,
                                             JSValueType type) {
  if (dest.isFloat()) {
    Label notInt32, end;
    asMasm().branchTestInt32(Assembler::NotEqual, src, &notInt32);
    convertInt32ToDouble(src.valueReg(), dest.fpu());
    ma_branch(&end);
    bind(&notInt32);
    unboxDouble(src, dest.fpu());
    bind(&end);
  } else {
    unboxNonDouble(src, dest.gpr(), type);
  }
}

void MacroAssemblerRiscv64Compat::boxDouble(FloatRegister src,
                                            const ValueOperand& dest,
                                            FloatRegister) {
  fmv_x_d(dest.valueReg(), src);
}

void MacroAssemblerRiscv64Compat::boxNonDouble(JSValueType type,
                                               Register src,
                                               const ValueOperand& dest) {
  MOZ_ASSERT(src != dest.valueReg());
  boxValue(type, src, dest.valueReg());
}

void MacroAssemblerRiscv64Compat::boolValueToDouble(const ValueOperand& operand,
                                                    FloatRegister dest) {
  UseScratchRegisterScope temps(this);
  Register ScratchRegister = temps.Acquire();
  convertBoolToInt32(operand.valueReg(), ScratchRegister);
  convertInt32ToDouble(ScratchRegister, dest);
}

void MacroAssemblerRiscv64Compat::int32ValueToDouble(
    const ValueOperand& operand,
    FloatRegister dest) {
  convertInt32ToDouble(operand.valueReg(), dest);
}

void MacroAssemblerRiscv64Compat::boolValueToFloat32(
    const ValueOperand& operand,
    FloatRegister dest) {
  UseScratchRegisterScope temps(this);
  Register ScratchRegister = temps.Acquire();
  convertBoolToInt32(operand.valueReg(), ScratchRegister);
  convertInt32ToFloat32(ScratchRegister, dest);
}

void MacroAssemblerRiscv64Compat::int32ValueToFloat32(
    const ValueOperand& operand,
    FloatRegister dest) {
  convertInt32ToFloat32(operand.valueReg(), dest);
}

void MacroAssemblerRiscv64Compat::loadConstantFloat32(float f,
                                                      FloatRegister dest) {
  ma_lis(dest, f);
}

void MacroAssemblerRiscv64Compat::loadInt32OrDouble(const Address& src,
                                                    FloatRegister dest) {
  Label notInt32, end;
  // If it's an int, convert it to double.
  UseScratchRegisterScope temps(this);
  Register ScratchRegister = temps.Acquire();
  Register SecondScratchReg = temps.Acquire();
  loadPtr(Address(src.base, src.offset), ScratchRegister);
  srli(SecondScratchReg, ScratchRegister, JSVAL_TAG_SHIFT);
  asMasm().branchTestInt32(Assembler::NotEqual, SecondScratchReg, &notInt32);
  loadPtr(Address(src.base, src.offset), SecondScratchReg);
  convertInt32ToDouble(SecondScratchReg, dest);
  ma_branch(&end);

  // Not an int, just load as double.
  bind(&notInt32);
  unboxDouble(src, dest);
  bind(&end);
}

void MacroAssemblerRiscv64Compat::loadInt32OrDouble(const BaseIndex& addr,
                                                    FloatRegister dest) {
  Label notInt32, end;

  UseScratchRegisterScope temps(this);
  Register ScratchRegister = temps.Acquire();
  Register SecondScratchReg = temps.Acquire();
  // If it's an int, convert it to double.
  computeScaledAddress(addr, SecondScratchReg);
  // Since we only have one scratch, we need to stomp over it with the tag.
  loadPtr(Address(SecondScratchReg, 0), ScratchRegister);
  srli(SecondScratchReg, ScratchRegister, JSVAL_TAG_SHIFT);
  asMasm().branchTestInt32(Assembler::NotEqual, SecondScratchReg, &notInt32);

  computeScaledAddress(addr, SecondScratchReg);
  loadPtr(Address(SecondScratchReg, 0), SecondScratchReg);
  convertInt32ToDouble(SecondScratchReg, dest);
  ma_branch(&end);

  // Not an int, just load as double.
  bind(&notInt32);
  // First, recompute the offset that had been stored in the scratch register
  // since the scratch register was overwritten loading in the type.
  computeScaledAddress(addr, SecondScratchReg);
  unboxDouble(Address(SecondScratchReg, 0), dest);
  bind(&end);
}

void MacroAssemblerRiscv64Compat::loadConstantDouble(double dp,
                                                     FloatRegister dest) {
  ma_lid(dest, dp);
}

Register MacroAssemblerRiscv64Compat::extractObject(const Address& address,
                                                    Register scratch) {
  loadPtr(Address(address.base, address.offset), scratch);
  ExtractBits(scratch, scratch, 0, JSVAL_TAG_SHIFT);
  return scratch;
}

Register MacroAssemblerRiscv64Compat::extractTag(const Address& address,
                                                 Register scratch) {
  loadPtr(Address(address.base, address.offset), scratch);
  ExtractBits(scratch, scratch, JSVAL_TAG_SHIFT, 64 - JSVAL_TAG_SHIFT);
  return scratch;
}

Register MacroAssemblerRiscv64Compat::extractTag(const BaseIndex& address,
                                                 Register scratch) {
  computeScaledAddress(address, scratch);
  return extractTag(Address(scratch, address.offset), scratch);
}

/////////////////////////////////////////////////////////////////
// X86/X64-common/ARM/LoongArch interface.
/////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////
// X86/X64-common/ARM/MIPS interface.
/////////////////////////////////////////////////////////////////
void MacroAssemblerRiscv64Compat::storeValue(ValueOperand val,
                                             const BaseIndex& dest) {
  UseScratchRegisterScope temps(this);
  Register ScratchRegister = temps.Acquire();
  computeScaledAddress(dest, ScratchRegister);
  storeValue(val, Address(ScratchRegister, dest.offset));
}

void MacroAssemblerRiscv64Compat::storeValue(JSValueType type,
                                             Register reg,
                                             BaseIndex dest) {
  UseScratchRegisterScope temps(this);
  Register ScratchRegister = temps.Acquire();
  Register SecondScratchReg = temps.Acquire();
  computeScaledAddress(dest, ScratchRegister);

  int32_t offset = dest.offset;
  if (!is_int12(offset)) {
    ma_li(SecondScratchReg, Imm32(offset));
    add(ScratchRegister, ScratchRegister, SecondScratchReg);
    offset = 0;
  }

  storeValue(type, reg, Address(ScratchRegister, offset));
}

void MacroAssemblerRiscv64Compat::storeValue(ValueOperand val,
                                             const Address& dest) {
  storePtr(val.valueReg(), Address(dest.base, dest.offset));
}

void MacroAssemblerRiscv64Compat::storeValue(JSValueType type,
                                             Register reg,
                                             Address dest) {
  UseScratchRegisterScope temps(this);
  Register SecondScratchReg = temps.Acquire();
  MOZ_ASSERT(dest.base != SecondScratchReg);

  if (type == JSVAL_TYPE_INT32 || type == JSVAL_TYPE_BOOLEAN) {
    store32(reg, dest);
    JSValueShiftedTag tag = (JSValueShiftedTag)JSVAL_TYPE_TO_SHIFTED_TAG(type);
    store32(((Imm64(tag)).secondHalf()), Address(dest.base, dest.offset + 4));
  } else {
    ma_li(SecondScratchReg, ImmTag(JSVAL_TYPE_TO_TAG(type)));
    slli(SecondScratchReg, SecondScratchReg, JSVAL_TAG_SHIFT);
    InsertBits(SecondScratchReg, reg, 0, JSVAL_TAG_SHIFT);
    storePtr(SecondScratchReg, Address(dest.base, dest.offset));
  }
}

void MacroAssemblerRiscv64Compat::storeValue(const Value& val, Address dest) {
  UseScratchRegisterScope temps(this);
  Register SecondScratchReg = temps.Acquire();
  if (val.isGCThing()) {
    writeDataRelocation(val);
    movWithPatch(ImmWord(val.asRawBits()), SecondScratchReg);
  } else {
    ma_li(SecondScratchReg, ImmWord(val.asRawBits()));
  }
  storePtr(SecondScratchReg, Address(dest.base, dest.offset));
}

void MacroAssemblerRiscv64Compat::storeValue(const Value& val, BaseIndex dest) {
  UseScratchRegisterScope temps(this);
  Register ScratchRegister = temps.Acquire();
  Register SecondScratchReg = temps.Acquire();
  computeScaledAddress(dest, ScratchRegister);

  int32_t offset = dest.offset;
  if (!is_int12(offset)) {
    ma_li(SecondScratchReg, Imm32(offset));
    add(ScratchRegister, ScratchRegister, SecondScratchReg);
    offset = 0;
  }
  storeValue(val, Address(ScratchRegister, offset));
}

void MacroAssemblerRiscv64Compat::loadValue(const BaseIndex& addr,
                                            ValueOperand val) {
  UseScratchRegisterScope temps(this);
  Register ScratchRegister = temps.Acquire();
  computeScaledAddress(addr, ScratchRegister);
  loadValue(Address(ScratchRegister, addr.offset), val);
}

void MacroAssemblerRiscv64Compat::loadValue(Address src, ValueOperand val) {
  loadPtr(Address(src.base, src.offset), val.valueReg());
}

void MacroAssemblerRiscv64Compat::tagValue(JSValueType type,
                                           Register payload,
                                           ValueOperand dest) {
  UseScratchRegisterScope temps(this);
  Register ScratchRegister = temps.Acquire();
  MOZ_ASSERT(dest.valueReg() != ScratchRegister);
  if (payload != dest.valueReg()) {
    mv(dest.valueReg(), payload);
  }
  ma_li(ScratchRegister, ImmTag(JSVAL_TYPE_TO_TAG(type)));
  InsertBits(dest.valueReg(), ScratchRegister, JSVAL_TAG_SHIFT,
             64 - JSVAL_TAG_SHIFT);
  if (type == JSVAL_TYPE_INT32 || type == JSVAL_TYPE_BOOLEAN) {
    InsertBits(dest.valueReg(), zero, 32, JSVAL_TAG_SHIFT - 32);
  }
}

void MacroAssemblerRiscv64Compat::pushValue(ValueOperand val) {
  // Allocate stack slots for Value. One for each.
  asMasm().subPtr(Imm32(sizeof(Value)), StackPointer);
  // Store Value
  storeValue(val, Address(StackPointer, 0));
}

void MacroAssemblerRiscv64Compat::pushValue(const Address& addr) {
  // Load value before allocate stack, addr.base may be is sp.
  UseScratchRegisterScope temps(this);
  Register ScratchRegister = temps.Acquire();
  loadPtr(Address(addr.base, addr.offset), ScratchRegister);
  ma_sub64(StackPointer, StackPointer, Imm32(sizeof(Value)));
  storePtr(ScratchRegister, Address(StackPointer, 0));
}

void MacroAssemblerRiscv64Compat::popValue(ValueOperand val) {
  ld(val.valueReg(), StackPointer, 0);
  ma_add64(StackPointer, StackPointer, Imm32(sizeof(Value)));
}

void MacroAssemblerRiscv64Compat::breakpoint(uint32_t value) {
  break_(value);
}

void MacroAssemblerRiscv64Compat::ensureDouble(const ValueOperand& source,
                                               FloatRegister dest,
                                               Label* failure) {
  Label isDouble, done;
  {
    ScratchTagScope tag(asMasm(), source);
    splitTagForTest(source, tag);
    asMasm().branchTestDouble(Assembler::Equal, tag, &isDouble);
    asMasm().branchTestInt32(Assembler::NotEqual, tag, failure);
  }
  UseScratchRegisterScope temps(this);
  Register ScratchRegister = temps.Acquire();
  unboxInt32(source, ScratchRegister);
  convertInt32ToDouble(ScratchRegister, dest);
  jump(&done);

  bind(&isDouble);
  unboxDouble(source, dest);

  bind(&done);
}

void MacroAssemblerRiscv64Compat::handleFailureWithHandlerTail(
    Label* profilerExitTail,
    Label* bailoutTail) {
  // Reserve space for exception information.
  int size = (sizeof(ResumeFromException) + ABIStackAlignment) &
             ~(ABIStackAlignment - 1);
  asMasm().subPtr(Imm32(size), StackPointer);
  mv(a0, StackPointer);  // Use a0 since it is a first function argument

  // Call the handler.
  using Fn = void (*)(ResumeFromException * rfe);
  asMasm().setupUnalignedABICall(a1);
  asMasm().passABIArg(a0);
  asMasm().callWithABI<Fn, HandleException>(
      MoveOp::GENERAL, CheckUnsafeCallWithABI::DontCheckHasExitFrame);

  Label entryFrame;
  Label catch_;
  Label finally;
  Label returnBaseline;
  Label returnIon;
  Label bailout;
  Label wasm;
  Label wasmCatch;

  // Already clobbered a0, so use it...
  load32(Address(StackPointer, ResumeFromException::offsetOfKind()), a0);
  asMasm().branch32(Assembler::Equal, a0,
                    Imm32(ExceptionResumeKind::EntryFrame), &entryFrame);
  asMasm().branch32(Assembler::Equal, a0, Imm32(ExceptionResumeKind::Catch),
                    &catch_);
  asMasm().branch32(Assembler::Equal, a0, Imm32(ExceptionResumeKind::Finally),
                    &finally);
  asMasm().branch32(Assembler::Equal, a0,
                    Imm32(ExceptionResumeKind::ForcedReturnBaseline),
                    &returnBaseline);
  asMasm().branch32(Assembler::Equal, a0,
                    Imm32(ExceptionResumeKind::ForcedReturnIon), &returnIon);
  asMasm().branch32(Assembler::Equal, a0, Imm32(ExceptionResumeKind::Bailout),
                    &bailout);
  asMasm().branch32(Assembler::Equal, a0, Imm32(ExceptionResumeKind::Wasm),
                    &wasm);
  asMasm().branch32(Assembler::Equal, a0, Imm32(ExceptionResumeKind::WasmCatch),
                    &wasmCatch);

  breakpoint();  // Invalid kind.

  // No exception handler. Load the error value, restore state and return from
  // the entry frame.
  bind(&entryFrame);
  asMasm().moveValue(MagicValue(JS_ION_ERROR), JSReturnOperand);
  loadPtr(Address(StackPointer, ResumeFromException::offsetOfFramePointer()),
          FramePointer);
  loadPtr(Address(StackPointer, ResumeFromException::offsetOfStackPointer()),
          StackPointer);

  // We're going to be returning by the ion calling convention
  ma_pop(ra);
  jump(ra);
  nop();

  // If we found a catch handler, this must be a baseline frame. Restore
  // state and jump to the catch block.
  bind(&catch_);
  loadPtr(Address(StackPointer, ResumeFromException::offsetOfTarget()), a0);
  loadPtr(Address(StackPointer, ResumeFromException::offsetOfFramePointer()),
          FramePointer);
  loadPtr(Address(StackPointer, ResumeFromException::offsetOfStackPointer()),
          StackPointer);
  jump(a0);

  // If we found a finally block, this must be a baseline frame. Push two
  // values expected by the finally block: the exception and BooleanValue(true).
  bind(&finally);
  ValueOperand exception = ValueOperand(a1);
  loadValue(Address(sp, ResumeFromException::offsetOfException()), exception);

  loadPtr(Address(sp, ResumeFromException::offsetOfTarget()), a0);
  loadPtr(Address(sp, ResumeFromException::offsetOfFramePointer()),
          FramePointer);
  loadPtr(Address(sp, ResumeFromException::offsetOfStackPointer()), sp);

  pushValue(exception);
  pushValue(BooleanValue(true));
  jump(a0);

  // Return BaselineFrame->returnValue() to the caller.
  // Used in debug mode and for GeneratorReturn.
  Label profilingInstrumentation;
  bind(&returnBaseline);
  loadPtr(Address(StackPointer, ResumeFromException::offsetOfFramePointer()),
          FramePointer);
  loadPtr(Address(StackPointer, ResumeFromException::offsetOfStackPointer()),
          StackPointer);
  loadValue(Address(FramePointer, BaselineFrame::reverseOffsetOfReturnValue()),
            JSReturnOperand);
  jump(&profilingInstrumentation);

  // Return the given value to the caller.
  bind(&returnIon);
  loadValue(Address(StackPointer, ResumeFromException::offsetOfException()),
            JSReturnOperand);
  loadPtr(Address(StackPointer, ResumeFromException::offsetOfFramePointer()),
          FramePointer);
  loadPtr(Address(StackPointer, ResumeFromException::offsetOfStackPointer()),
          StackPointer);

  // If profiling is enabled, then update the lastProfilingFrame to refer to
  // caller frame before returning. This code is shared by ForcedReturnIon
  // and ForcedReturnBaseline.
  bind(&profilingInstrumentation);
  {
    Label skipProfilingInstrumentation;
    // Test if profiler enabled.
    AbsoluteAddress addressOfEnabled(
        asMasm().runtime()->geckoProfiler().addressOfEnabled());
    asMasm().branch32(Assembler::Equal, addressOfEnabled, Imm32(0),
                      &skipProfilingInstrumentation);
    jump(profilerExitTail);
    bind(&skipProfilingInstrumentation);
  }

  mv(StackPointer, FramePointer);
  pop(FramePointer);
  ret();

  // If we are bailing out to baseline to handle an exception, jump to
  // the bailout tail stub. Load 1 (true) in ReturnReg to indicate success.
  bind(&bailout);
  loadPtr(Address(sp, ResumeFromException::offsetOfBailoutInfo()), a2);
  loadPtr(Address(StackPointer, ResumeFromException::offsetOfStackPointer()),
          StackPointer);
  ma_li(ReturnReg, Imm32(1));
  jump(bailoutTail);

  // If we are throwing and the innermost frame was a wasm frame, reset SP and
  // FP; SP is pointing to the unwound return address to the wasm entry, so
  // we can just ret().
  bind(&wasm);
  loadPtr(Address(StackPointer, ResumeFromException::offsetOfFramePointer()),
          FramePointer);
  loadPtr(Address(StackPointer, ResumeFromException::offsetOfStackPointer()),
          StackPointer);
  ret();

  // Found a wasm catch handler, restore state and jump to it.
  bind(&wasmCatch);
  loadPtr(Address(sp, ResumeFromException::offsetOfTarget()), a1);
  loadPtr(Address(StackPointer, ResumeFromException::offsetOfFramePointer()),
          FramePointer);
  loadPtr(Address(StackPointer, ResumeFromException::offsetOfStackPointer()),
          StackPointer);
  jump(a1);
}

CodeOffset MacroAssemblerRiscv64Compat::toggledJump(Label* label) {
  CodeOffset ret(nextOffset().getOffset());
  jump(label);
  return ret;
}

CodeOffset MacroAssemblerRiscv64Compat::toggledCall(JitCode* target,
                                                    bool enabled) {
  BufferOffset bo = nextOffset();
  CodeOffset offset(bo.getOffset());
  UseScratchRegisterScope temps(this);
  Register ScratchRegister = temps.Acquire();
  addPendingJump(bo, ImmPtr(target->raw()), RelocationKind::JITCODE);
  ma_liPatchable(ScratchRegister, ImmPtr(target->raw()));
  if (enabled) {
    jalr(ScratchRegister);
    nop();
  } else {
    nop();
    nop();
  }
  MOZ_ASSERT_IF(!oom(), nextOffset().getOffset() - offset.offset() ==
                            ToggledCallSize(nullptr));
  return offset;
}

void MacroAssembler::subFromStackPtr(Imm32 imm32) {
  if (imm32.value) {
    asMasm().subPtr(imm32, StackPointer);
  }
}

void MacroAssembler::clampDoubleToUint8(FloatRegister input, Register output) {
  MOZ_CRASH();
}

//{{{ check_macroassembler_style
// ===============================================================
// MacroAssembler high-level usage.
bool MacroAssembler::convertUInt64ToDoubleNeedsTemp() {
  return false;
}
CodeOffset MacroAssembler::call(Label* label) {
  jump(label);
  return CodeOffset(currentOffset());
}
CodeOffset MacroAssembler::call(Register reg) {
  jalr(reg, 0);
  return CodeOffset(currentOffset());
}
CodeOffset MacroAssembler::call(wasm::SymbolicAddress target) {
  movePtr(target, CallReg);
  return call(CallReg);
}
CodeOffset MacroAssembler::callWithPatch() {
  MOZ_CRASH();
  return CodeOffset(currentOffset());
}
CodeOffset MacroAssembler::farJumpWithPatch() {
  UseScratchRegisterScope temps(this);
  Register scratch = temps.Acquire();
  Register scratch2 = temps.Acquire();
  auipc(scratch, 0);
  lw(scratch2, scratch, 4 * sizeof(Instr));
  add(scratch, scratch, scratch2);
  jr(scratch, 0);
  // Allocate space which will be patched by patchFarJump().
  CodeOffset farJump(currentOffset());
  spew(".space 32bit initValue 0xffff ffff");
  emit(UINT32_MAX);
  return farJump;
}
CodeOffset MacroAssembler::moveNearAddressWithPatch(Register dest) {
  return movWithPatch(ImmPtr(nullptr), dest);
}
CodeOffset MacroAssembler::nopPatchableToCall() {
  // riscv64
  nop();  // lui(rd, (int32_t)high_20);
  nop();  // addi(rd, rd, low_12);  // 31 bits in rd.
  nop();  // slli(rd, rd, 11);      // Space for next 11 bis
  nop();  // ori(rd, rd, b11);      // 11 bits are put in. 42 bit in rd
  nop();  // slli(rd, rd, 6);       // Space for next 6 bits
  nop();  // ori(rd, rd, a6);       // 6 bits are put in. 48 bis in rd
  nop();  // jirl
  return CodeOffset(currentOffset());
}
CodeOffset MacroAssembler::wasmTrapInstruction() {
  CodeOffset offset(currentOffset());
  ebreak();  // TODO: as_teq(zero, zero, WASM_TRAP)
  return offset;
}
size_t MacroAssembler::PushRegsInMaskSizeInBytes(LiveRegisterSet set) {
  return set.gprs().size() * sizeof(intptr_t) + set.fpus().getPushSizeInBytes();
}
template <typename T>
void MacroAssembler::branchValueIsNurseryCellImpl(Condition cond,
                                                  const T& value,
                                                  Register temp,
                                                  Label* label) {
  MOZ_ASSERT(cond == Assembler::Equal || cond == Assembler::NotEqual);
  MOZ_ASSERT(temp != InvalidReg);
  Label done;
  branchTestGCThing(Assembler::NotEqual, value,
                    cond == Assembler::Equal ? &done : label);

  unboxGCThingForGCBarrier(value, temp);
  orPtr(Imm32(gc::ChunkMask), temp);
  loadPtr(Address(temp, gc::ChunkStoreBufferOffsetFromLastByte), temp);
  branchPtr(InvertCondition(cond), temp, zero, label);

  bind(&done);
}

template <typename T>
void MacroAssembler::storeUnboxedValue(const ConstantOrRegister& value,
                                       MIRType valueType,
                                       const T& dest,
                                       MIRType slotType) {
  if (valueType == MIRType::Double) {
    boxDouble(value.reg().typedReg().fpu(), dest);
    return;
  }

  // For known integers and booleans, we can just store the unboxed value if
  // the slot has the same type.
  if ((valueType == MIRType::Int32 || valueType == MIRType::Boolean) &&
      slotType == valueType) {
    if (value.constant()) {
      Value val = value.value();
      if (valueType == MIRType::Int32) {
        store32(Imm32(val.toInt32()), dest);
      } else {
        store32(Imm32(val.toBoolean() ? 1 : 0), dest);
      }
    } else {
      store32(value.reg().typedReg().gpr(), dest);
    }
    return;
  }

  if (value.constant()) {
    storeValue(value.value(), dest);
  } else {
    storeValue(ValueTypeFromMIRType(valueType), value.reg().typedReg().gpr(),
               dest);
  }
}

template void MacroAssembler::storeUnboxedValue(const ConstantOrRegister& value,
                                                MIRType valueType,
                                                const Address& dest,
                                                MIRType slotType);
template void MacroAssembler::storeUnboxedValue(
    const ConstantOrRegister& value,
    MIRType valueType,
    const BaseObjectElementIndex& dest,
    MIRType slotType);

// ===============================================================
// Jit Frames.

uint32_t MacroAssembler::pushFakeReturnAddress(Register scratch) {
  CodeLabel cl;

  ma_li(scratch, &cl);
  Push(scratch);
  bind(&cl);
  uint32_t retAddr = currentOffset();

  addCodeLabel(cl);
  return retAddr;
}

void MacroAssembler::atomicEffectOp64(const Synchronization&,
                                      AtomicOp,
                                      Register64,
                                      const Address&,
                                      Register64) {
  MOZ_CRASH();
}
void MacroAssembler::atomicEffectOp64(const Synchronization&,
                                      AtomicOp,
                                      Register64,
                                      const BaseIndex&,
                                      Register64) {
  MOZ_CRASH();
}
void MacroAssembler::atomicEffectOpJS(Scalar::Type,
                                      const Synchronization&,
                                      AtomicOp,
                                      Register,
                                      const Address&,
                                      Register,
                                      Register,
                                      Register) {
  MOZ_CRASH();
}
void MacroAssembler::atomicEffectOpJS(Scalar::Type,
                                      const Synchronization&,
                                      AtomicOp,
                                      Register,
                                      const BaseIndex&,
                                      Register,
                                      Register,
                                      Register) {
  MOZ_CRASH();
}
void MacroAssembler::atomicExchange64(const Synchronization&,
                                      const Address&,
                                      Register64,
                                      Register64) {
  MOZ_CRASH();
}
void MacroAssembler::atomicExchange64(const Synchronization&,
                                      const BaseIndex&,
                                      Register64,
                                      Register64) {
  MOZ_CRASH();
}
void MacroAssembler::atomicExchangeJS(Scalar::Type,
                                      const Synchronization&,
                                      const Address&,
                                      Register,
                                      Register,
                                      Register,
                                      Register,
                                      Register,
                                      AnyRegister) {
  MOZ_CRASH();
}
void MacroAssembler::atomicExchangeJS(Scalar::Type,
                                      const Synchronization&,
                                      const BaseIndex&,
                                      Register,
                                      Register,
                                      Register,
                                      Register,
                                      Register,
                                      AnyRegister) {
  MOZ_CRASH();
}
void MacroAssembler::atomicExchange(Scalar::Type,
                                    const Synchronization&,
                                    const Address&,
                                    Register,
                                    Register,
                                    Register,
                                    Register,
                                    Register) {
  MOZ_CRASH();
}
void MacroAssembler::atomicExchange(Scalar::Type,
                                    const Synchronization&,
                                    const BaseIndex&,
                                    Register,
                                    Register,
                                    Register,
                                    Register,
                                    Register) {
  MOZ_CRASH();
}
void MacroAssembler::atomicFetchOp64(const Synchronization&,
                                     AtomicOp,
                                     Register64,
                                     const Address&,
                                     Register64,
                                     Register64) {
  MOZ_CRASH();
}
void MacroAssembler::atomicFetchOp64(const Synchronization&,
                                     AtomicOp,
                                     Register64,
                                     const BaseIndex&,
                                     Register64,
                                     Register64) {
  MOZ_CRASH();
}
void MacroAssembler::atomicFetchOpJS(Scalar::Type,
                                     const Synchronization&,
                                     AtomicOp,
                                     Register,
                                     const Address&,
                                     Register,
                                     Register,
                                     Register,
                                     Register,
                                     AnyRegister) {
  MOZ_CRASH();
}
void MacroAssembler::atomicFetchOpJS(Scalar::Type,
                                     const Synchronization&,
                                     AtomicOp,
                                     Register,
                                     const BaseIndex&,
                                     Register,
                                     Register,
                                     Register,
                                     Register,
                                     AnyRegister) {
  MOZ_CRASH();
}
void MacroAssembler::atomicFetchOp(Scalar::Type,
                                   const Synchronization&,
                                   AtomicOp,
                                   Register,
                                   const Address&,
                                   Register,
                                   Register,
                                   Register,
                                   Register) {
  MOZ_CRASH();
}
void MacroAssembler::atomicFetchOp(Scalar::Type,
                                   const Synchronization&,
                                   AtomicOp,
                                   Register,
                                   const BaseIndex&,
                                   Register,
                                   Register,
                                   Register,
                                   Register) {
  MOZ_CRASH();
}
void MacroAssembler::branchPtrInNurseryChunk(Condition cond,
                                             Register ptr,
                                             Register temp,
                                             Label* label) {
  MOZ_ASSERT(cond == Assembler::Equal || cond == Assembler::NotEqual);
  MOZ_ASSERT(ptr != temp);
  MOZ_ASSERT(ptr != ScratchRegister);  // Both may be used internally.
  MOZ_ASSERT(temp != ScratchRegister);
  MOZ_ASSERT(temp != InvalidReg);

  movePtr(ptr, temp);
  orPtr(Imm32(gc::ChunkMask), temp);
  branchPtr(InvertCondition(cond),
            Address(temp, gc::ChunkStoreBufferOffsetFromLastByte), zero, label);
}
void MacroAssembler::branchTestValue(Condition cond,
                                     const ValueOperand& lhs,
                                     const Value& rhs,
                                     Label* label) {
  MOZ_ASSERT(cond == Equal || cond == NotEqual);
  UseScratchRegisterScope temps(this);
  Register scratch = temps.Acquire();
  MOZ_ASSERT(lhs.valueReg() != scratch);
  moveValue(rhs, ValueOperand(scratch));
  ma_b(lhs.valueReg(), scratch, label, cond);
}
void MacroAssembler::branchValueIsNurseryCell(Condition cond,
                                              const Address& address,
                                              Register temp,
                                              Label* label) {
  branchValueIsNurseryCellImpl(cond, address, temp, label);
}

void MacroAssembler::branchValueIsNurseryCell(Condition cond,
                                              ValueOperand value,
                                              Register temp,
                                              Label* label) {
  branchValueIsNurseryCellImpl(cond, value, temp, label);
}
void MacroAssembler::call(const Address& addr) {
  loadPtr(addr, CallReg);
  call(CallReg);
}
void MacroAssembler::call(ImmPtr target) {
  BufferOffset bo = m_buffer.nextOffset();
  addPendingJump(bo, target, RelocationKind::HARDCODED);
  ma_call(target);
}
void MacroAssembler::call(ImmWord target) {
  call(ImmPtr((void*)target.value));
}

void MacroAssembler::call(JitCode* c) {
  UseScratchRegisterScope temps(this);
  Register scratch = temps.Acquire();
  BufferOffset bo = m_buffer.nextOffset();
  addPendingJump(bo, ImmPtr(c->raw()), RelocationKind::JITCODE);
  ma_liPatchable(scratch, ImmPtr(c->raw()));
  callJitNoProfiler(scratch);
}

void MacroAssembler::callWithABIPre(uint32_t* stackAdjust, bool callFromWasm) {
  MOZ_ASSERT(inCall_);
  uint32_t stackForCall = abiArgs_.stackBytesConsumedSoFar();

  // Reserve place for $ra.
  stackForCall += sizeof(intptr_t);

  if (dynamicAlignment_) {
    stackForCall += ComputeByteAlignment(stackForCall, ABIStackAlignment);
  } else {
    uint32_t alignmentAtPrologue = callFromWasm ? sizeof(wasm::Frame) : 0;
    stackForCall += ComputeByteAlignment(
        stackForCall + framePushed() + alignmentAtPrologue, ABIStackAlignment);
  }

  *stackAdjust = stackForCall;
  reserveStack(stackForCall);

  // Save $ra because call is going to clobber it. Restore it in
  // callWithABIPost. NOTE: This is needed for calls from SharedIC.
  // Maybe we can do this differently.
  storePtr(ra, Address(StackPointer, stackForCall - sizeof(intptr_t)));

  // Position all arguments.
  {
    enoughMemory_ &= moveResolver_.resolve();
    if (!enoughMemory_) {
      return;
    }

    MoveEmitter emitter(asMasm());
    emitter.emit(moveResolver_);
    emitter.finish();
  }

  assertStackAlignment(ABIStackAlignment);
}

void MacroAssembler::callWithABIPost(uint32_t stackAdjust,
                                     MoveOp::Type result,
                                     bool callFromWasm) {
  // Restore ra value (as stored in callWithABIPre()).
  loadPtr(Address(StackPointer, stackAdjust - sizeof(intptr_t)), ra);

  if (dynamicAlignment_) {
    // Restore sp value from stack (as stored in setupUnalignedABICall()).
    loadPtr(Address(StackPointer, stackAdjust), StackPointer);
    // Use adjustFrame instead of freeStack because we already restored sp.
    adjustFrame(-stackAdjust);
  } else {
    freeStack(stackAdjust);
  }

#ifdef DEBUG
  MOZ_ASSERT(inCall_);
  inCall_ = false;
#endif
}

void MacroAssembler::callWithABINoProfiler(Register fun, MoveOp::Type result) {
  // Load the callee in scratch2, no instruction between the movePtr and
  // call should clobber it. Note that we can't use fun because it may be
  // one of the IntArg registers clobbered before the call.
  movePtr(fun, CallReg);

  uint32_t stackAdjust;
  callWithABIPre(&stackAdjust);
  call(CallReg);
  callWithABIPost(stackAdjust, result);
}

void MacroAssembler::callWithABINoProfiler(const Address& fun,
                                           MoveOp::Type result) {
  // Load the callee in scratch2, as above.
  loadPtr(fun, CallReg);

  uint32_t stackAdjust;
  callWithABIPre(&stackAdjust);
  call(CallReg);
  callWithABIPost(stackAdjust, result);
}

void MacroAssembler::ceilDoubleToInt32(FloatRegister src,
                                       Register dest,
                                       Label* fail) {
  UseScratchRegisterScope temps(this);
  Register scratch = temps.Acquire();
  Ceil_w_d(dest, src, scratch);
  ma_b(scratch, Imm32(1), fail, NotEqual);
}

void MacroAssembler::ceilFloat32ToInt32(FloatRegister src,
                                       Register dest,
                                       Label* fail) {
  UseScratchRegisterScope temps(this);
  Register scratch = temps.Acquire();
  Ceil_w_s(dest, src, scratch);
  ma_b(scratch, Imm32(1), fail, NotEqual);
}
void MacroAssembler::comment(const char* msg) {
  Assembler::comment(msg);
}

void MacroAssembler::compareExchange64(const Synchronization&,
                                       const Address&,
                                       Register64,
                                       Register64,
                                       Register64) {
  MOZ_CRASH();
}
void MacroAssembler::compareExchange64(const Synchronization&,
                                       const BaseIndex&,
                                       Register64,
                                       Register64,
                                       Register64) {
  MOZ_CRASH();
}
void MacroAssembler::compareExchangeJS(Scalar::Type,
                                       const Synchronization&,
                                       const Address&,
                                       Register,
                                       Register,
                                       Register,
                                       Register,
                                       Register,
                                       Register,
                                       AnyRegister) {
  MOZ_CRASH();
}
void MacroAssembler::compareExchangeJS(Scalar::Type,
                                       const Synchronization&,
                                       const BaseIndex&,
                                       Register,
                                       Register,
                                       Register,
                                       Register,
                                       Register,
                                       Register,
                                       AnyRegister) {
  MOZ_CRASH();
}
void MacroAssembler::compareExchange(Scalar::Type,
                                     const Synchronization&,
                                     const Address&,
                                     Register,
                                     Register,
                                     Register,
                                     Register,
                                     Register,
                                     Register) {
  MOZ_CRASH();
}
void MacroAssembler::compareExchange(Scalar::Type,
                                     const Synchronization&,
                                     const BaseIndex&,
                                     Register,
                                     Register,
                                     Register,
                                     Register,
                                     Register,
                                     Register) {
  MOZ_CRASH();
}
void MacroAssembler::convertInt64ToDouble(Register64 src, FloatRegister dest) {
  fcvt_d_l(dest, src.scratchReg());
}
void MacroAssembler::convertInt64ToFloat32(Register64 src, FloatRegister dest) {
  fcvt_s_l(dest, src.scratchReg());
}
void MacroAssembler::convertIntPtrToDouble(Register src, FloatRegister dest) {
  fcvt_d_l(dest, src);
}
void MacroAssembler::convertUInt64ToDouble(Register64 src,
                                           FloatRegister dest,
                                           Register tmp) {
  fcvt_d_lu(dest, src.scratchReg());
}
void MacroAssembler::convertUInt64ToFloat32(Register64 src,
                                           FloatRegister dest,
                                           Register tmp) {
  fcvt_s_lu(dest, src.scratchReg());
}
void MacroAssembler::copySignDouble(FloatRegister,
                                    FloatRegister,
                                    FloatRegister) {
  MOZ_CRASH();
}
void MacroAssembler::enterFakeExitFrameForWasm(Register cxreg,
                                               Register scratch,
                                               ExitFrameType type) {
  enterFakeExitFrame(cxreg, scratch, type);
}
void MacroAssembler::flexibleDivMod32(Register rhs,
                                      Register srcDest,
                                      Register remOutput,
                                      bool isUnsigned,
                                      const LiveRegisterSet&) {
  if (isUnsigned) {
    ma_modu32(remOutput, srcDest, rhs);
    ma_divu32(srcDest, srcDest, rhs);
  } else {
    ma_mod32(remOutput, srcDest, rhs);
    ma_div32(srcDest, srcDest, rhs);
  }
}
void MacroAssembler::flexibleQuotient32(Register rhs,
                                        Register srcDest,
                                        bool isUnsigned,
                                        const LiveRegisterSet&) {
  quotient32(rhs, srcDest, isUnsigned);
}

void MacroAssembler::flexibleRemainder32(Register rhs,
                                         Register srcDest,
                                         bool isUnsigned,
                                         const LiveRegisterSet&) {
  remainder32(rhs, srcDest, isUnsigned);
}

void MacroAssembler::floorDoubleToInt32(FloatRegister src,
                                        Register dest,
                                        Label* fail) {
  UseScratchRegisterScope temps(this);
  Register scratch = temps.Acquire();
  Floor_w_d(dest, src, scratch);
  ma_b(scratch, Imm32(1), fail, NotEqual);
}
void MacroAssembler::floorFloat32ToInt32(FloatRegister src,
                                         Register dest,
                                         Label* fail) {
  UseScratchRegisterScope temps(this);
  Register scratch = temps.Acquire();
  Floor_w_s(dest, src, scratch);
  ma_b(scratch, Imm32(1), fail, NotEqual);
}
void MacroAssembler::flush() {}
void MacroAssembler::loadStoreBuffer(Register ptr, Register buffer) {
  if (ptr != buffer) {
    movePtr(ptr, buffer);
  }
  orPtr(Imm32(gc::ChunkMask), buffer);
  loadPtr(Address(buffer, gc::ChunkStoreBufferOffsetFromLastByte), buffer);
}

void MacroAssembler::moveValue(const TypedOrValueRegister& src,
                               const ValueOperand& dest) {
  if (src.hasValue()) {
    moveValue(src.valueReg(), dest);
    return;
  }

  MIRType type = src.type();
  AnyRegister reg = src.typedReg();

  if (!IsFloatingPointType(type)) {
    boxNonDouble(ValueTypeFromMIRType(type), reg.gpr(), dest);
    return;
  }

  ScratchDoubleScope fpscratch(asMasm());
  FloatRegister scratch = fpscratch;
  FloatRegister freg = reg.fpu();
  if (type == MIRType::Float32) {
    convertFloat32ToDouble(freg, scratch);
    freg = scratch;
  }
  boxDouble(freg, dest, scratch);
}
void MacroAssembler::moveValue(const ValueOperand& src,
                               const ValueOperand& dest) {
  if (src == dest) {
    return;
  }
  movePtr(src.valueReg(), dest.valueReg());
}

void MacroAssembler::moveValue(const Value& src, const ValueOperand& dest) {
  if (!src.isGCThing()) {
    ma_li(dest.valueReg(), ImmWord(src.asRawBits()));
    return;
  }

  writeDataRelocation(src);
  movWithPatch(ImmWord(src.asRawBits()), dest.valueReg());
}
void MacroAssembler::nearbyIntDouble(RoundingMode,
                                     FloatRegister,
                                     FloatRegister) {
  MOZ_CRASH("not supported on this platform");
}
void MacroAssembler::nearbyIntFloat32(RoundingMode,
                                      FloatRegister,
                                      FloatRegister) {
  MOZ_CRASH("not supported on this platform");
}

void MacroAssembler::oolWasmTruncateCheckF32ToI32(FloatRegister input,
                                                  Register output,
                                                  TruncFlags flags,
                                                  wasm::BytecodeOffset off,
                                                  Label* rejoin) {
  Label notNaN;
  UseScratchRegisterScope temps(this);
  Register scratch = temps.Acquire();
  CompareIsNotNanF32(scratch, input, input);
  ma_branch(&notNaN, Equal, scratch, Operand(1));
  wasmTrap(wasm::Trap::InvalidConversionToInteger, off);
  bind(&notNaN);

  Label isOverflow;
  const float two_31 = -float(INT32_MIN);
  ScratchFloat32Scope fpscratch(*this);
  if (flags & TRUNC_UNSIGNED) {
    loadConstantFloat32(two_31 * 2, fpscratch);
    ma_compareF32(scratch, Assembler::DoubleGreaterThanOrEqual, input,
                  fpscratch);
    ma_branch(&isOverflow, Equal, scratch, Operand(1));
    loadConstantFloat32(-1.0f, fpscratch);
    ma_compareF32(scratch, Assembler::DoubleGreaterThan, input, fpscratch);
    ma_b(scratch, Imm32(1), rejoin, Equal);
  } else {
    loadConstantFloat32(two_31, fpscratch);
    ma_compareF32(scratch, Assembler::DoubleGreaterThanOrEqual, input,
                  fpscratch);
    ma_branch(&isOverflow, Equal, scratch, Operand(1));
    loadConstantFloat32(-two_31, fpscratch);
    ma_compareF32(scratch, Assembler::DoubleGreaterThanOrEqual, input,
                  fpscratch);
    ma_b(scratch, Imm32(1), rejoin, Equal);
  }
  bind(&isOverflow);
  wasmTrap(wasm::Trap::IntegerOverflow, off);
}

void MacroAssembler::oolWasmTruncateCheckF64ToI32(FloatRegister input,
                                                  Register output,
                                                  TruncFlags flags,
                                                  wasm::BytecodeOffset off,
                                                  Label* rejoin) {
  Label notNaN;
  UseScratchRegisterScope temps(this);
  Register scratch = temps.Acquire();
  CompareIsNotNanF64(scratch, input, input);
  ma_branch(&notNaN, Equal, scratch, Operand(1));
  wasmTrap(wasm::Trap::InvalidConversionToInteger, off);
  bind(&notNaN);

  Label isOverflow;
  const double two_31 = -double(INT32_MIN);
  ScratchDoubleScope fpscratch(*this);
  if (flags & TRUNC_UNSIGNED) {
    loadConstantDouble(two_31 * 2, fpscratch);
    ma_compareF64(scratch, Assembler::DoubleGreaterThanOrEqual, input,
                  fpscratch);
    ma_branch(&isOverflow, Equal, scratch, Operand(1));
    loadConstantDouble(-1.0, fpscratch);
    ma_compareF64(scratch, Assembler::DoubleGreaterThan, input, fpscratch);
    ma_b(scratch, Imm32(1), rejoin, Equal);
  } else {
    loadConstantDouble(two_31, fpscratch);
    ma_compareF64(scratch, Assembler::DoubleGreaterThanOrEqual, input,
                  fpscratch);
    ma_branch(&isOverflow, Equal, scratch, Operand(1));
    loadConstantDouble(-two_31 - 1, fpscratch);
    ma_compareF64(scratch, Assembler::DoubleGreaterThan, input, fpscratch);
    ma_b(scratch, Imm32(1), rejoin, Equal);
  }
  bind(&isOverflow);
  wasmTrap(wasm::Trap::IntegerOverflow, off);
}

void MacroAssembler::oolWasmTruncateCheckF32ToI64(FloatRegister input,
                                                  Register64 output,
                                                  TruncFlags flags,
                                                  wasm::BytecodeOffset off,
                                                  Label* rejoin) {
  Label notNaN;
  UseScratchRegisterScope temps(this);
  Register scratch = temps.Acquire();
  CompareIsNotNanF32(scratch, input, input);
  ma_branch(&notNaN, Equal, scratch, Operand(1));
  wasmTrap(wasm::Trap::InvalidConversionToInteger, off);
  bind(&notNaN);

  Label isOverflow;
  const float two_63 = -float(INT64_MIN);
  ScratchFloat32Scope fpscratch(*this);
  if (flags & TRUNC_UNSIGNED) {
    loadConstantFloat32(two_63 * 2, fpscratch);
    ma_compareF32(scratch, Assembler::DoubleGreaterThanOrEqual, input,
                  fpscratch);
    ma_branch(&isOverflow, Equal, scratch, Operand(1));
    loadConstantFloat32(-1.0f, fpscratch);
    ma_compareF32(scratch, Assembler::DoubleGreaterThan, input, fpscratch);
    ma_b(scratch, Imm32(1), rejoin, Equal);
  } else {
    loadConstantFloat32(two_63, fpscratch);
    ma_compareF32(scratch, Assembler::DoubleGreaterThanOrEqual, input,
                  fpscratch);
    ma_branch(&isOverflow, Equal, scratch, Operand(1));
    loadConstantFloat32(-two_63, fpscratch);
    ma_compareF32(scratch, Assembler::DoubleGreaterThanOrEqual, input,
                  fpscratch);
    ma_b(scratch, Imm32(1), rejoin, Equal);
  }
  bind(&isOverflow);
  wasmTrap(wasm::Trap::IntegerOverflow, off);
}

void MacroAssembler::oolWasmTruncateCheckF64ToI64(FloatRegister input,
                                                  Register64 output,
                                                  TruncFlags flags,
                                                  wasm::BytecodeOffset off,
                                                  Label* rejoin) {
  Label notNaN;
  UseScratchRegisterScope temps(this);
  Register scratch = temps.Acquire();
  CompareIsNotNanF64(scratch, input, input);
  ma_branch(&notNaN, Equal, scratch, Operand(1));
  wasmTrap(wasm::Trap::InvalidConversionToInteger, off);
  bind(&notNaN);

  Label isOverflow;
  const double two_63 = -double(INT64_MIN);
  ScratchDoubleScope fpscratch(*this);
  if (flags & TRUNC_UNSIGNED) {
    loadConstantDouble(two_63 * 2, fpscratch);
    ma_compareF64(scratch, Assembler::DoubleGreaterThanOrEqual, input,
                  fpscratch);
    ma_branch(&isOverflow, Equal, scratch, Operand(1));
    loadConstantDouble(-1.0, fpscratch);
    ma_compareF64(scratch, Assembler::DoubleGreaterThan, input, fpscratch);
    ma_b(scratch, Imm32(1), rejoin, Equal);
  } else {
    loadConstantDouble(two_63, fpscratch);
    ma_compareF64(scratch, Assembler::DoubleGreaterThanOrEqual, input,
                  fpscratch);
    ma_branch(&isOverflow, Equal, scratch, Operand(1));
    loadConstantDouble(-two_63, fpscratch);
    ma_compareF64(scratch, Assembler::DoubleGreaterThan, input, fpscratch);
    ma_b(scratch, Imm32(1), rejoin, Equal);
  }
  bind(&isOverflow);
  wasmTrap(wasm::Trap::IntegerOverflow, off);
}
void MacroAssembler::patchCallToNop(uint8_t* call) {
  uint32_t* p = reinterpret_cast<uint32_t*>(call) - 7;
  *reinterpret_cast<Instr*>(p) = kNopByte;
  *reinterpret_cast<Instr*>(p + 1) = kNopByte;
  *reinterpret_cast<Instr*>(p + 2) = kNopByte;
  *reinterpret_cast<Instr*>(p + 3) = kNopByte;
  *reinterpret_cast<Instr*>(p + 4) = kNopByte;
  *reinterpret_cast<Instr*>(p + 5) = kNopByte;
  *reinterpret_cast<Instr*>(p + 6) = kNopByte;
}

void MacroAssembler::patchCall(uint32_t callerOffset, uint32_t calleeOffset) {
  BufferOffset call(callerOffset - 1 * sizeof(uint32_t));

  int32_t offset = BufferOffset(calleeOffset).getOffset() - call.getOffset();
  if (is_int12(offset)) {
    MOZ_CRASH();
  } else {
    uint32_t u32Offset = callerOffset - 4 * sizeof(uint32_t);
    uint32_t* u32 =
        reinterpret_cast<uint32_t*>(editSrc(BufferOffset(u32Offset)));
    *u32 = calleeOffset - callerOffset;
  }
}
void MacroAssembler::patchFarJump(CodeOffset farJump, uint32_t targetOffset) {
  uint32_t* u32 =
      reinterpret_cast<uint32_t*>(editSrc(BufferOffset(farJump.offset())));
  MOZ_ASSERT(*u32 == UINT32_MAX);
  *u32 = targetOffset - farJump.offset();
}
void MacroAssembler::patchNearAddressMove(CodeLocationLabel loc,
                                          CodeLocationLabel target) {
  PatchDataWithValueCheck(loc, ImmPtr(target.raw()), ImmPtr(nullptr));
}
void MacroAssembler::patchNopToCall(uint8_t* call, uint8_t* target) {
  uint32_t* p = reinterpret_cast<uint32_t*>(call) - 7;
  Assembler::WriteLoad64Instructions((Instruction*)p, ScratchRegister, (uint64_t)target);
  Instr jalr_ = JALR | (ra.code() << kRdShift) | (0x0 << kFunct3Shift) |
                (ScratchRegister.code() << kRs1Shift) | (0x0 << kImm12Shift);
  *reinterpret_cast<Instr*>(p + 6) = jalr_;
}
void MacroAssembler::Pop(Register reg) {
  ma_pop(reg);
  adjustFrame(-int32_t(sizeof(intptr_t)));
}

void MacroAssembler::Pop(FloatRegister f) {
  ma_pop(f);
  adjustFrame(-int32_t(sizeof(double)));
}

void MacroAssembler::Pop(const ValueOperand& val) {
  popValue(val);
  adjustFrame(-int32_t(sizeof(Value)));
}

void MacroAssembler::PopRegsInMaskIgnore(LiveRegisterSet set,
                                         LiveRegisterSet ignore) {
  int32_t diff =
      set.gprs().size() * sizeof(intptr_t) + set.fpus().getPushSizeInBytes();
  const int32_t reserved = diff;

  for (GeneralRegisterBackwardIterator iter(set.gprs()); iter.more(); ++iter) {
    diff -= sizeof(intptr_t);
    if (!ignore.has(*iter)) {
      loadPtr(Address(StackPointer, diff), *iter);
    }
  }

#ifdef ENABLE_WASM_SIMD
#error "Needs more careful logic if SIMD is enabled"
#endif

  for (FloatRegisterBackwardIterator iter(set.fpus().reduceSetForPush());
       iter.more(); ++iter) {
    diff -= sizeof(double);
    if (!ignore.has(*iter)) {
      loadDouble(Address(StackPointer, diff), *iter);
    }
  }
  MOZ_ASSERT(diff == 0);
  freeStack(reserved);
}

void MacroAssembler::pushReturnAddress() {
  push(ra);
}

void MacroAssembler::popReturnAddress() {
  pop(ra);
}
void MacroAssembler::PopStackPtr() {
  loadPtr(Address(StackPointer, 0), StackPointer);
  adjustFrame(-int32_t(sizeof(intptr_t)));
}
void MacroAssembler::PushBoxed(FloatRegister) {
  MOZ_CRASH();
}

void MacroAssembler::Push(Register reg) {
  ma_push(reg);
  adjustFrame(int32_t(sizeof(intptr_t)));
}

void MacroAssembler::Push(const Imm32 imm) {
  UseScratchRegisterScope temps(this);
  Register scratch = temps.Acquire();
  ma_li(scratch, imm);
  ma_push(scratch);
  adjustFrame(int32_t(sizeof(intptr_t)));
}

void MacroAssembler::Push(const ImmWord imm) {
  UseScratchRegisterScope temps(this);
  Register scratch = temps.Acquire();
  ma_li(scratch, imm);
  ma_push(scratch);
  adjustFrame(int32_t(sizeof(intptr_t)));
}

void MacroAssembler::Push(const ImmPtr imm) {
  Push(ImmWord(uintptr_t(imm.value)));
}

void MacroAssembler::Push(const ImmGCPtr ptr) {
  UseScratchRegisterScope temps(this);
  Register scratch = temps.Acquire();
  ma_li(scratch, ptr);
  ma_push(scratch);
  adjustFrame(int32_t(sizeof(intptr_t)));
}

void MacroAssembler::Push(FloatRegister f) {
  ma_push(f);
  adjustFrame(int32_t(sizeof(double)));
}

void MacroAssembler::PushRegsInMask(LiveRegisterSet set) {
  int32_t diff =
      set.gprs().size() * sizeof(intptr_t) + set.fpus().getPushSizeInBytes();
  const int32_t reserved = diff;

  reserveStack(reserved);
  for (GeneralRegisterBackwardIterator iter(set.gprs()); iter.more(); ++iter) {
    diff -= sizeof(intptr_t);
    storePtr(*iter, Address(StackPointer, diff));
  }

#ifdef ENABLE_WASM_SIMD
#error "Needs more careful logic if SIMD is enabled"
#endif

  for (FloatRegisterBackwardIterator iter(set.fpus().reduceSetForPush());
       iter.more(); ++iter) {
    diff -= sizeof(double);
    storeDouble(*iter, Address(StackPointer, diff));
  }
  MOZ_ASSERT(diff == 0);
}

void MacroAssembler::roundDoubleToInt32(FloatRegister src,
                                        Register dest,
                                        FloatRegister temp,
                                        Label* fail) {
  UseScratchRegisterScope temps(this);
  Register scratch = temps.Acquire();
  Round_w_d(dest, src, scratch);
  ma_b(scratch, Imm32(1), fail, NotEqual);
}
void MacroAssembler::roundFloat32ToInt32(FloatRegister src,
                                         Register dest,
                                         FloatRegister temp,
                                         Label* fail) {
  UseScratchRegisterScope temps(this);
  Register scratch = temps.Acquire();
  Round_w_s(dest, src, scratch);
  ma_b(scratch, Imm32(1), fail, NotEqual);
}
void MacroAssembler::setupUnalignedABICall(Register scratch) {
  MOZ_ASSERT(!IsCompilingWasm(), "wasm should only use aligned ABI calls");
  setupNativeABICall();
  dynamicAlignment_ = true;

  or_(scratch, StackPointer, zero);

  // Force sp to be aligned
  asMasm().subPtr(Imm32(sizeof(uintptr_t)), StackPointer);
  ma_and(StackPointer, StackPointer, Imm32(~(ABIStackAlignment - 1)));
  storePtr(scratch, Address(StackPointer, 0));
}
void MacroAssembler::shiftIndex32AndAdd(Register, int, Register) {
  MOZ_CRASH();
}
void MacroAssembler::speculationBarrier() {
  MOZ_CRASH();
}
void MacroAssembler::storeRegsInMask(LiveRegisterSet, Address, Register) {
  MOZ_CRASH();
}
void MacroAssembler::truncDoubleToInt32(FloatRegister src,
                                        Register dest,
                                        Label* fail) {
  UseScratchRegisterScope temps(this);
  Register scratch = temps.Acquire();
  Round_w_s(dest, src, scratch);
  ma_b(scratch, Imm32(1), fail, NotEqual);
}
void MacroAssembler::truncFloat32ToInt32(FloatRegister src,
                                         Register dest,
                                         Label* fail) {
  UseScratchRegisterScope temps(this);
  Register scratch = temps.Acquire();
  Round_w_s(dest, src, scratch);
  ma_b(scratch, Imm32(1), fail, NotEqual);
}
void MacroAssembler::wasmAtomicEffectOp(const wasm::MemoryAccessDesc&,
                                        AtomicOp,
                                        Register,
                                        const Address&,
                                        Register,
                                        Register,
                                        Register) {
  MOZ_CRASH();
}
void MacroAssembler::wasmAtomicEffectOp(const wasm::MemoryAccessDesc&,
                                        AtomicOp,
                                        Register,
                                        const BaseIndex&,
                                        Register,
                                        Register,
                                        Register) {
  MOZ_CRASH();
}
void MacroAssembler::wasmAtomicExchange64(const wasm::MemoryAccessDesc&,
                                          const Address&,
                                          Register64,
                                          Register64) {
  MOZ_CRASH();
}
void MacroAssembler::wasmAtomicExchange64(const wasm::MemoryAccessDesc&,
                                          const BaseIndex&,
                                          Register64,
                                          Register64) {
  MOZ_CRASH();
}
void MacroAssembler::wasmAtomicExchange(const wasm::MemoryAccessDesc&,
                                        const Address&,
                                        Register,
                                        Register,
                                        Register,
                                        Register,
                                        Register) {
  MOZ_CRASH();
}
void MacroAssembler::wasmAtomicExchange(const wasm::MemoryAccessDesc&,
                                        const BaseIndex&,
                                        Register,
                                        Register,
                                        Register,
                                        Register,
                                        Register) {
  MOZ_CRASH();
}
void MacroAssembler::wasmAtomicFetchOp64(const wasm::MemoryAccessDesc&,
                                         AtomicOp,
                                         Register64,
                                         const Address&,
                                         Register64,
                                         Register64) {
  MOZ_CRASH();
}
void MacroAssembler::wasmAtomicFetchOp64(const wasm::MemoryAccessDesc&,
                                         AtomicOp,
                                         Register64,
                                         const BaseIndex&,
                                         Register64,
                                         Register64) {
  MOZ_CRASH();
}
void MacroAssembler::wasmAtomicFetchOp(const wasm::MemoryAccessDesc&,
                                       AtomicOp,
                                       Register,
                                       const Address&,
                                       Register,
                                       Register,
                                       Register,
                                       Register) {
  MOZ_CRASH();
}
void MacroAssembler::wasmAtomicFetchOp(const wasm::MemoryAccessDesc&,
                                       AtomicOp,
                                       Register,
                                       const BaseIndex&,
                                       Register,
                                       Register,
                                       Register,
                                       Register) {
  MOZ_CRASH();
}
void MacroAssembler::wasmBoundsCheck32(Condition, Register, Address, Label*) {
  MOZ_CRASH();
}
void MacroAssembler::wasmBoundsCheck32(Condition, Register, Register, Label*) {
  MOZ_CRASH();
}
void MacroAssembler::wasmBoundsCheck64(Condition, Register64, Address, Label*) {
  MOZ_CRASH();
}
void MacroAssembler::wasmBoundsCheck64(Condition,
                                       Register64,
                                       Register64,
                                       Label*) {
  MOZ_CRASH();
}
void MacroAssembler::wasmCompareExchange64(const wasm::MemoryAccessDesc&,
                                           const Address&,
                                           Register64,
                                           Register64,
                                           Register64) {
  MOZ_CRASH();
}
void MacroAssembler::wasmCompareExchange64(const wasm::MemoryAccessDesc&,
                                           const BaseIndex&,
                                           Register64,
                                           Register64,
                                           Register64) {
  MOZ_CRASH();
}
void MacroAssembler::wasmCompareExchange(const wasm::MemoryAccessDesc&,
                                         const Address&,
                                         Register,
                                         Register,
                                         Register,
                                         Register,
                                         Register,
                                         Register) {
  MOZ_CRASH();
}
void MacroAssembler::wasmCompareExchange(const wasm::MemoryAccessDesc&,
                                         const BaseIndex&,
                                         Register,
                                         Register,
                                         Register,
                                         Register,
                                         Register,
                                         Register) {
  MOZ_CRASH();
}
void MacroAssembler::wasmLoad(const wasm::MemoryAccessDesc&,
                              Register,
                              Register,
                              Register,
                              AnyRegister) {
  MOZ_CRASH();
}
void MacroAssembler::wasmLoadI64(const wasm::MemoryAccessDesc&,
                                 Register,
                                 Register,
                                 Register,
                                 Register64) {
  MOZ_CRASH();
}
void MacroAssembler::wasmStore(const wasm::MemoryAccessDesc&,
                               AnyRegister,
                               Register,
                               Register,
                               Register) {
  MOZ_CRASH();
}
void MacroAssembler::wasmStoreI64(const wasm::MemoryAccessDesc&,
                                  Register64,
                                  Register,
                                  Register,
                                  Register) {
  MOZ_CRASH();
}
void MacroAssembler::wasmTruncateDoubleToInt32(FloatRegister,
                                               Register,
                                               bool,
                                               Label*) {
  MOZ_CRASH();
}
void MacroAssembler::wasmTruncateDoubleToInt64(FloatRegister,
                                               Register64,
                                               bool,
                                               Label*,
                                               Label*,
                                               FloatRegister) {
  MOZ_CRASH();
}
void MacroAssembler::wasmTruncateDoubleToUInt32(FloatRegister,
                                                Register,
                                                bool,
                                                Label*) {
  MOZ_CRASH();
}
void MacroAssembler::wasmTruncateDoubleToUInt64(FloatRegister,
                                                Register64,
                                                bool,
                                                Label*,
                                                Label*,
                                                FloatRegister) {
  MOZ_CRASH();
}
void MacroAssembler::wasmTruncateFloat32ToInt32(FloatRegister,
                                                Register,
                                                bool,
                                                Label*) {
  MOZ_CRASH();
}
void MacroAssembler::wasmTruncateFloat32ToInt64(FloatRegister,
                                                Register64,
                                                bool,
                                                Label*,
                                                Label*,
                                                FloatRegister) {
  MOZ_CRASH();
}
void MacroAssembler::wasmTruncateFloat32ToUInt32(FloatRegister,
                                                 Register,
                                                 bool,
                                                 Label*) {
  MOZ_CRASH();
}
void MacroAssembler::wasmTruncateFloat32ToUInt64(FloatRegister,
                                                 Register64,
                                                 bool,
                                                 Label*,
                                                 Label*,
                                                 FloatRegister) {
  MOZ_CRASH();
}
void MacroAssembler::widenInt32(Register) {
  MOZ_CRASH();
}

//}}} check_macroassembler_style

// This method generates lui, dsll and ori instruction block that can be
// modified by UpdateLoad64Value, either during compilation (eg.
// Assembler::bind), or during execution (eg. jit::PatchJump).
void MacroAssemblerRiscv64::ma_liPatchable(Register dest, Imm32 imm) {
  return ma_liPatchable(dest, ImmWord(uintptr_t(imm.value)));
}

void MacroAssemblerRiscv64::ma_liPatchable(Register dest, ImmPtr imm) {
  return ma_liPatchable(dest, ImmWord(uintptr_t(imm.value)));
}

void MacroAssemblerRiscv64::ma_liPatchable(Register dest,
                                           ImmWord imm,
                                           LiFlags flags) {
  if (Li64 == flags) {
    m_buffer.ensureSpace(8 * sizeof(uint32_t));
    li_constant(dest, imm.value);
  } else {
    m_buffer.ensureSpace(6 * sizeof(uint32_t));
    li_ptr(dest, imm.value);
  }
}

void MacroAssemblerRiscv64::ma_li(Register dest, ImmGCPtr ptr) {
  writeDataRelocation(ptr);
  ma_liPatchable(dest, ImmPtr(ptr.value));
}
void MacroAssemblerRiscv64::ma_li(Register dest, Imm32 imm) {
  RV_li(dest, imm.value);
}
void MacroAssemblerRiscv64::ma_li(Register dest, CodeLabel* label) {
  BufferOffset bo = m_buffer.nextOffset();
  JitSpew(JitSpew_Codegen, ".load CodeLabel %p", label);
  ma_liPatchable(dest, ImmWord(/* placeholder */ 0));
  label->patchAt()->bind(bo.getOffset());
  label->setLinkMode(CodeLabel::MoveImmediate);
}
void MacroAssemblerRiscv64::ma_li(Register dest, ImmWord imm) {
  RV_li(dest, imm.value);
}

// Shortcut for when we know we're transferring 32 bits of data.
void MacroAssemblerRiscv64::ma_pop(Register r) {
  ld(r, StackPointer, 0);
  addi(StackPointer, StackPointer, sizeof(intptr_t));
}

void MacroAssemblerRiscv64::ma_push(Register r) {
  if (r == sp) {
    UseScratchRegisterScope temps(this);
    Register ScratchRegister = temps.Acquire();
    // Pushing sp requires one more instruction.
    mv(ScratchRegister, sp);
    r = ScratchRegister;
  }

  addi(StackPointer, StackPointer, (int32_t) - sizeof(intptr_t));
  sd(r, StackPointer, 0);
}

// multiplies.  For now, there are only few that we care about.
void MacroAssemblerRiscv64::ma_mul32TestOverflow(Register rd,
                                                 Register rj,
                                                 Register rk,
                                                 Label* overflow) {
  UseScratchRegisterScope temps(this);
  Register ScratchRegister = temps.Acquire();
  MulOverflow32(rd, rj, rk, ScratchRegister);
  ma_b(ScratchRegister, Register(zero), overflow, Assembler::NotEqual);
}
void MacroAssemblerRiscv64::ma_mul32TestOverflow(Register rd,
                                                 Register rj,
                                                 Imm32 imm,
                                                 Label* overflow) {
  UseScratchRegisterScope temps(this);
  Register ScratchRegister = temps.Acquire();
  MulOverflow32(rd, rj, Operand(imm.value), ScratchRegister);
  ma_b(ScratchRegister, Register(zero), overflow, Assembler::NotEqual);
}

void MacroAssemblerRiscv64::ma_mulPtrTestOverflow(Register rd,
                                                  Register rj,
                                                  Register rk,
                                                  Label* overflow) {
  UseScratchRegisterScope temps(this);
  Register scratch = temps.Acquire();
  Register scratch2 = temps.Acquire();
  MOZ_ASSERT(rd != scratch);

  if (rd == rj) {
    or_(scratch, rj, zero);
    rj = scratch;
    rk = (rd == rk) ? rj : rk;
  } else if (rd == rk) {
    or_(scratch, rk, zero);
    rk = scratch;
  }

  mul(rd, rj, rk);
  mulh(scratch, rj, rk);
  srai(scratch2, rd, 63);
  ma_b(scratch, Register(scratch2), overflow, Assembler::NotEqual);
}

// MulOverflow32 sets overflow register to zero if no overflow occured
void MacroAssemblerRiscv64::MulOverflow32(Register dst,
                                          Register left,
                                          const Operand& right,
                                          Register overflow) {
  UseScratchRegisterScope temps(this);
  BlockTrampolinePoolScope block_trampoline_pool(this);
  Register right_reg;
  Register scratch = temps.Acquire();
  Register scratch2 = temps.Acquire();
  if (right.is_imm()) {
    ma_li(scratch, right.immediate());
    right_reg = scratch;
  } else {
    MOZ_ASSERT(right.is_reg());
    right_reg = right.rm();
  }

  MOZ_ASSERT(left != scratch2 && right_reg != scratch2 && dst != scratch2 &&
             overflow != scratch2);
  MOZ_ASSERT(overflow != left && overflow != right_reg);
  sext_w(overflow, left);
  sext_w(scratch2, right_reg);

  mul(overflow, overflow, scratch2);
  sext_w(dst, overflow);
  xor_(overflow, overflow, dst);
}

int32_t MacroAssemblerRiscv64::GetOffset(int32_t offset,
                                         Label* L,
                                         OffsetSize bits) {
  if (L) {
    offset = branch_offset_helper(L, bits);
  } else {
    MOZ_ASSERT(is_intn(offset, bits));
  }
  return offset;
}

bool MacroAssemblerRiscv64::CalculateOffset(Label* L,
                                            int32_t* offset,
                                            OffsetSize bits) {
  if (!is_near(L, bits))
    return false;
  *offset = GetOffset(*offset, L, bits);
  return true;
}

void MacroAssemblerRiscv64::BranchShortHelper(int32_t offset, Label* L) {
  MOZ_ASSERT(L == nullptr || offset == 0);
  offset = GetOffset(offset, L, OffsetSize::kOffset21);
  Assembler::j(offset);
}

bool MacroAssemblerRiscv64::BranchShortHelper(int32_t offset,
                                              Label* L,
                                              Condition cond,
                                              Register rs,
                                              const Operand& rt) {
  MOZ_ASSERT(L == nullptr || offset == 0);
  MOZ_ASSERT(rt.is_reg() || rt.is_imm());
  UseScratchRegisterScope temps(this);
  BlockTrampolinePoolScope block_trampoline_pool(this);
  Register scratch = Register();
  if (rt.is_imm()) {
    scratch = temps.Acquire();
    ma_li(scratch, Imm32(rt.immediate()));
  } else {
    MOZ_ASSERT(rt.is_reg());
    scratch = rt.rm();
  }
  {
    BlockTrampolinePoolScope block_trampoline_pool(this);
    switch (cond) {
      case Always:
        if (!CalculateOffset(L, &offset, OffsetSize::kOffset21))
          return false;
        Assembler::j(offset);
        EmitConstPoolWithJumpIfNeeded();
        break;
      case Equal:
        // rs == rt
        if (rt.is_reg() && rs == rt.rm()) {
          if (!CalculateOffset(L, &offset, OffsetSize::kOffset21))
            return false;
          Assembler::j(offset);
        } else {
          if (!CalculateOffset(L, &offset, OffsetSize::kOffset13))
            return false;
          Assembler::beq(rs, scratch, offset);
        }
        break;
      case NotEqual:
        // rs != rt
        if (rt.is_reg() && rs == rt.rm()) {
          break;  // No code needs to be emitted
        } else {
          if (!CalculateOffset(L, &offset, OffsetSize::kOffset13))
            return false;
          Assembler::bne(rs, scratch, offset);
        }
        break;

      // Signed comparison.
      case GreaterThan:
        // rs > rt
        if (rt.is_reg() && rs == rt.rm()) {
          break;  // No code needs to be emitted.
        } else {
          if (!CalculateOffset(L, &offset, OffsetSize::kOffset13))
            return false;
          Assembler::bgt(rs, scratch, offset);
        }
        break;
      case GreaterThanOrEqual:
        // rs >= rt
        if (rt.is_reg() && rs == rt.rm()) {
          if (!CalculateOffset(L, &offset, OffsetSize::kOffset21))
            return false;
          Assembler::j(offset);
        } else {
          if (!CalculateOffset(L, &offset, OffsetSize::kOffset13))
            return false;
          Assembler::bge(rs, scratch, offset);
        }
        break;
      case LessThan:
        // rs < rt
        if (rt.is_reg() && rs == rt.rm()) {
          break;  // No code needs to be emitted.
        } else {
          if (!CalculateOffset(L, &offset, OffsetSize::kOffset13))
            return false;
          Assembler::blt(rs, scratch, offset);
        }
        break;
      case LessThanOrEqual:
        // rs <= rt
        if (rt.is_reg() && rs == rt.rm()) {
          if (!CalculateOffset(L, &offset, OffsetSize::kOffset21))
            return false;
          Assembler::j(offset);
        } else {
          if (!CalculateOffset(L, &offset, OffsetSize::kOffset13))
            return false;
          Assembler::ble(rs, scratch, offset);
        }
        break;

      // Unsigned comparison.
      case Above:
        // rs > rt
        if (rt.is_reg() && rs == rt.rm()) {
          break;  // No code needs to be emitted.
        } else {
          if (!CalculateOffset(L, &offset, OffsetSize::kOffset13))
            return false;
          Assembler::bgtu(rs, scratch, offset);
        }
        break;
      case AboveOrEqual:
        // rs >= rt
        if (rt.is_reg() && rs == rt.rm()) {
          if (!CalculateOffset(L, &offset, OffsetSize::kOffset21))
            return false;
          Assembler::j(offset);
        } else {
          if (!CalculateOffset(L, &offset, OffsetSize::kOffset13))
            return false;
          Assembler::bgeu(rs, scratch, offset);
        }
        break;
      case Below:
        // rs < rt
        if (rt.is_reg() && rs == rt.rm()) {
          break;  // No code needs to be emitted.
        } else {
          if (!CalculateOffset(L, &offset, OffsetSize::kOffset13))
            return false;
          bltu(rs, scratch, offset);
        }
        break;
      case BelowOrEqual:
        // rs <= rt
        if (rt.is_reg() && rs == rt.rm()) {
          if (!CalculateOffset(L, &offset, OffsetSize::kOffset21))
            return false;
          Assembler::j(offset);
        } else {
          if (!CalculateOffset(L, &offset, OffsetSize::kOffset13))
            return false;
          Assembler::bleu(rs, scratch, offset);
        }
        break;
      default:
        MOZ_CRASH("UNREACHABLE");
    }
  }

  CheckTrampolinePoolQuick(1);
  return true;
}

// BRANCH_ARGS_CHECK checks that conditional jump arguments are correct.
#define BRANCH_ARGS_CHECK(cond, rs, rt)                           \
  MOZ_ASSERT((cond == Always && rs == zero && rt.rm() == zero) || \
             (cond != Always && (rs != zero || rt.rm() != zero)))

bool MacroAssemblerRiscv64::BranchShortCheck(int32_t offset,
                                             Label* L,
                                             Condition cond,
                                             Register rs,
                                             const Operand& rt) {
  BRANCH_ARGS_CHECK(cond, rs, rt);

  if (!L) {
    MOZ_ASSERT(is_int13(offset));
    return BranchShortHelper(offset, nullptr, cond, rs, rt);
  } else {
    MOZ_ASSERT(offset == 0);
    return BranchShortHelper(0, L, cond, rs, rt);
  }
}

void MacroAssemblerRiscv64::BranchShort(Label* L) {
  BranchShortHelper(0, L);
}

void MacroAssemblerRiscv64::BranchShort(int32_t offset,
                                        Condition cond,
                                        Register rs,
                                        const Operand& rt) {
  BranchShortCheck(offset, nullptr, cond, rs, rt);
}

void MacroAssemblerRiscv64::BranchShort(Label* L,
                                        Condition cond,
                                        Register rs,
                                        const Operand& rt) {
  BranchShortCheck(0, L, cond, rs, rt);
}

void MacroAssemblerRiscv64::BranchLong(Label* L) {
  // Generate position independent long branch.
  BlockTrampolinePoolScope block_trampoline_pool(this);
  int32_t imm;
  imm = branch_long_offset(L);
  GenPCRelativeJump(t6, imm);
  EmitConstPoolWithJumpIfNeeded();
}

void MacroAssemblerRiscv64::ma_branch(Label* L,
                                      Condition cond,
                                      Register rs,
                                      const Operand& rt,
                                      JumpKind jumpKind) {
  if (L->used()) {
    if (jumpKind == ShortJump && BranchShortCheck(0, L, cond, rs, rt)) {
      return;
    }
    if (cond != Always) {
      Label skip;
      Condition neg_cond = NegateCondition(cond);
      BranchShort(&skip, neg_cond, rs, rt);
      BranchLong(L);
      bind(&skip);
    } else {
      BranchLong(L);
      EmitConstPoolWithJumpIfNeeded();
    }
  } else {
    if (is_trampoline_emitted() && jumpKind == LongJump) {
      if (cond != Always) {
        Label skip;
        Condition neg_cond = NegateCondition(cond);
        ma_branch(&skip, neg_cond, rs, rt);
        BranchLong(L);
        bind(&skip);
      } else {
        BranchLong(L);
        EmitConstPoolWithJumpIfNeeded();
      }
    } else {
      BranchShort(L, cond, rs, rt);
    }
  }
}

// Branches when done from within riscv code.
void MacroAssemblerRiscv64::ma_b(Register lhs,
                                 Address addr,
                                 Label* label,
                                 Condition c,
                                 JumpKind jumpKind) {
  ScratchRegisterScope scratch(asMasm());
  MOZ_ASSERT(lhs != scratch);
  ma_load(scratch, addr, SizeDouble);
  ma_b(lhs, Register(scratch), label, c, jumpKind);
}

void MacroAssemblerRiscv64::ma_b(Register lhs,
                                 ImmPtr imm,
                                 Label* l,
                                 Condition c,
                                 JumpKind jumpKind) {
  asMasm().ma_b(lhs, ImmWord(uintptr_t(imm.value)), l, c, jumpKind);
}

// Branches when done from within loongarch-specific code.
void MacroAssemblerRiscv64::ma_b(Register lhs,
                                 ImmWord imm,
                                 Label* label,
                                 Condition c,
                                 JumpKind jumpKind) {
  UseScratchRegisterScope temps(this);
  Register scratch = temps.Acquire();
  MOZ_ASSERT(lhs != scratch);
  ma_li(scratch, imm);
  ma_b(lhs, Register(scratch), label, c, jumpKind);
}

void MacroAssemblerRiscv64::ma_b(Register lhs,
                                 Imm32 imm,
                                 Label* label,
                                 Condition c,
                                 JumpKind jumpKind) {
  UseScratchRegisterScope temps(this);
  Register scratch = temps.Acquire();
  MOZ_ASSERT(lhs != scratch);
  ma_li(scratch, imm);
  ma_b(lhs, Register(scratch), label, c, jumpKind);
}

void MacroAssemblerRiscv64::ma_b(Address addr,
                                 Imm32 imm,
                                 Label* label,
                                 Condition c,
                                 JumpKind jumpKind) {
  UseScratchRegisterScope temps(this);
  Register scratch2 = temps.Acquire();
  ma_load(scratch2, addr);
  ma_b(Register(scratch2), imm, label, c, jumpKind);
}

void MacroAssemblerRiscv64::ma_b(Register lhs,
                                 Register rhs,
                                 Label* label,
                                 Condition c,
                                 JumpKind jumpKind) {
  switch (c) {
    case Equal:
    case NotEqual:
      ma_branch(label, c, lhs, rhs, jumpKind);
      break;
    case Always:
      ma_branch(label, c, zero, Operand(zero), jumpKind);
      break;
    case Zero:
      MOZ_ASSERT(lhs == rhs);
      ma_branch(label, Equal, lhs, Operand(zero), jumpKind);
      break;
    case NonZero:
      MOZ_ASSERT(lhs == rhs);
      ma_branch(label, NotEqual, lhs, Operand(zero), jumpKind);
      break;
    case Signed:
      MOZ_ASSERT(lhs == rhs);
      ma_branch(label, GreaterThan, lhs, Operand(zero), jumpKind);
      break;
    case NotSigned:
      MOZ_ASSERT(lhs == rhs);
      ma_branch(label, LessThan, lhs, Operand(zero), jumpKind);
      break;
    default: {
      ma_branch(label, c, lhs, rhs, jumpKind);
      break;
    }
  }
}

void MacroAssemblerRiscv64::ExtractBits(Register rt,
                                        Register rs,
                                        uint16_t pos,
                                        uint16_t size,
                                        bool sign_extend) {
#if JS_CODEGEN_RISCV64
  MOZ_ASSERT(pos < 64 && 0 < size && size <= 64 && 0 < pos + size &&
             pos + size <= 64);
  slli(rt, rs, 64 - (pos + size));
  if (sign_extend) {
    srai(rt, rt, 64 - size);
  } else {
    srli(rt, rt, 64 - size);
  }
#elif JS_CODEGEN_RISCV32
  MOZ_ASSERT(pos < 32);
  MOZ_ASSERT(size > 0);
  MOZ_ASSERT(size <= 32);
  MOZ_ASSERT((pos + size) > 0);
  MOZ_ASSERT((pos + size) <= 32);
  slli(rt, rs, 32 - (pos + size));
  if (sign_extend) {
    srai(rt, rt, 32 - size);
  } else {
    srli(rt, rt, 32 - size);
  }
#endif
}

void MacroAssemblerRiscv64::InsertBits(Register dest,
                                       Register source,
                                       int pos,
                                       int size) {
#if JS_CODEGEN_RISCV64
  MOZ_ASSERT(size < 64);
#elif JS_CODEGEN_RISCV32
  MOZ_ASSERT(size < 32);
#endif
  UseScratchRegisterScope temps(this);
  Register mask = temps.Acquire();
  BlockTrampolinePoolScope block_trampoline_pool(this);
  Register source_ = temps.Acquire();
  // Create a mask of the length=size.
  ma_li(mask, Imm32(1));
  slli(mask, mask, size);
  addi(mask, mask, -1);
  and_(source_, mask, source);
  slli(source_, source_, pos);
  // Make a mask containing 0's. 0's start at "pos" with length=size.
  slli(mask, mask, pos);
  not_(mask, mask);
  // cut area for insertion of source.
  and_(dest, mask, dest);
  // insert source
  or_(dest, dest, source_);
}

void MacroAssemblerRiscv64::InsertBits(Register dest,
                                       Register source,
                                       Register pos,
                                       int size) {
#if JS_CODEGEN_RISCV64
  MOZ_ASSERT(size < 64);
#elif JS_CODEGEN_RISCV32
  MOZ_ASSERT(size < 32);
#endif
  UseScratchRegisterScope temps(this);
  Register mask = temps.Acquire();
  BlockTrampolinePoolScope block_trampoline_pool(this);
  Register source_ = temps.Acquire();
  // Create a mask of the length=size.
  ma_li(mask, Imm32(1));
  slli(mask, mask, size);
  addi(mask, mask, -1);
  and_(source_, mask, source);
  sll(source_, source_, pos);
  // Make a mask containing 0's. 0's start at "pos" with length=size.
  sll(mask, mask, pos);
  not_(mask, mask);
  // cut area for insertion of source.
  and_(dest, mask, dest);
  // insert source
  or_(dest, dest, source_);
}

void MacroAssemblerRiscv64::ma_add32(Register rd, Register rs, Operand rt) {
  if (rt.is_imm()) {
    if (is_int12(rt.immediate())) {
      addiw(rd, rs, static_cast<int32_t>(rt.immediate()));
    } else if ((-4096 <= rt.immediate() && rt.immediate() <= -2049) ||
               (2048 <= rt.immediate() && rt.immediate() <= 4094)) {
      addiw(rd, rs, rt.immediate() / 2);
      addiw(rd, rd, rt.immediate() - (rt.immediate() / 2));
    } else {
      // li handles the relocation.
      UseScratchRegisterScope temps(this);
      Register scratch = temps.Acquire();
      BlockTrampolinePoolScope block_trampoline_pool(this);
      ma_li(scratch, rt.immediate());
      addw(rd, rs, scratch);
    }
  } else {
    MOZ_ASSERT(rt.is_reg());
    addw(rd, rs, rt.rm());
  }
}

void MacroAssemblerRiscv64::ma_add64(Register rd, Register rs, Operand rt) {
  if (rt.is_imm()) {
    if (is_int12(rt.immediate())) {
      addi(rd, rs, static_cast<int32_t>(rt.immediate()));
    } else if ((-4096 <= rt.immediate() && rt.immediate() <= -2049) ||
               (2048 <= rt.immediate() && rt.immediate() <= 4094)) {
      addi(rd, rs, rt.immediate() / 2);
      addi(rd, rd, rt.immediate() - (rt.immediate() / 2));
    } else {
      // li handles the relocation.
      UseScratchRegisterScope temps(this);
      Register scratch = temps.Acquire();
      BlockTrampolinePoolScope block_trampoline_pool(this);
      ma_li(scratch, rt.immediate());
      add(rd, rs, scratch);
    }
  } else {
    MOZ_ASSERT(rt.is_reg());
    add(rd, rs, rt.rm());
  }
}

void MacroAssemblerRiscv64::ma_sub32(Register rd, Register rs, Operand rt) {
  if (rt.is_imm()) {
    if (is_int12(-rt.immediate())) {
      addiw(rd, rs,
            static_cast<int32_t>(
                -rt.immediate()));  // No subi instr, use addi(x, y, -imm).
    } else if ((-4096 <= -rt.immediate() && -rt.immediate() <= -2049) ||
               (2048 <= -rt.immediate() && -rt.immediate() <= 4094)) {
      addiw(rd, rs, -rt.immediate() / 2);
      addiw(rd, rd, -rt.immediate() - (-rt.immediate() / 2));
    } else {
      // li handles the relocation.
      UseScratchRegisterScope temps(this);
      Register scratch = temps.Acquire();
      ma_li(scratch, rt.immediate());
      subw(rd, rs, scratch);
    }
  } else {
    MOZ_ASSERT(rt.is_reg());
    subw(rd, rs, rt.rm());
  }
}

void MacroAssemblerRiscv64::ma_sub64(Register rd, Register rs, Operand rt) {
  if (rt.is_imm()) {
    if (is_int12(-rt.immediate())) {
      addi(rd, rs,
           static_cast<int32_t>(
               -rt.immediate()));  // No subi instr, use addi(x, y, -imm).
    } else if ((-4096 <= -rt.immediate() && -rt.immediate() <= -2049) ||
               (2048 <= -rt.immediate() && -rt.immediate() <= 4094)) {
      addi(rd, rs, -rt.immediate() / 2);
      addi(rd, rd, -rt.immediate() - (-rt.immediate() / 2));
    } else {
      // li handles the relocation.
      UseScratchRegisterScope temps(this);
      Register scratch = temps.Acquire();
      ma_li(scratch, rt.immediate());
      sub(rd, rs, scratch);
    }
  } else {
    MOZ_ASSERT(rt.is_reg());
    sub(rd, rs, rt.rm());
  }
}

void MacroAssemblerRiscv64::ma_and(Register rd, Register rs, Operand rt) {
  if (rt.is_imm()) {
    if (is_int12(rt.immediate())) {
      andi(rd, rs, rt.immediate());
    } else {
      UseScratchRegisterScope temps(this);
      Register ScratchRegister = temps.Acquire();
      ma_li(ScratchRegister, rt.immediate());
      and_(rd, rs, ScratchRegister);
    }
  } else {
    MOZ_ASSERT(rt.is_reg());
    and_(rd, rs, rt.rm());
  }
}

void MacroAssemblerRiscv64::ma_or(Register rd, Register rs, Operand rt) {
  if (rt.is_imm()) {
    if (is_int12(rt.immediate())) {
      ori(rd, rs, rt.immediate());
    } else {
      UseScratchRegisterScope temps(this);
      Register ScratchRegister = temps.Acquire();
      ma_li(ScratchRegister, rt.immediate());
      or_(rd, rs, ScratchRegister);
    }
  } else {
    MOZ_ASSERT(rt.is_reg());
    or_(rd, rs, rt.rm());
  }
}

void MacroAssemblerRiscv64::ma_xor(Register rd, Register rs, Operand rt) {
  if (rt.is_imm()) {
    if (is_int12(rt.immediate())) {
      xori(rd, rs, rt.immediate());
    } else {
      UseScratchRegisterScope temps(this);
      Register ScratchRegister = temps.Acquire();
      ma_li(ScratchRegister, rt.immediate());
      xor_(rd, rs, ScratchRegister);
    }
  } else {
    MOZ_ASSERT(rt.is_reg());
    xor_(rd, rs, rt.rm());
  }
}

void MacroAssemblerRiscv64::ma_nor(Register rd, Register rs, Operand rt) {
  if (rt.is_imm()) {
    UseScratchRegisterScope temps(this);
    Register ScratchRegister = temps.Acquire();
    ma_li(ScratchRegister, rt.immediate());
    nor(rd, rs, ScratchRegister);
  } else {
    MOZ_ASSERT(rt.is_reg());
    nor(rd, rs, rt.rm());
  }
}

void MacroAssemblerRiscv64::ma_div32(Register rd, Register rs, Operand rt) {
  if (rt.is_imm()) {
    UseScratchRegisterScope temps(this);
    Register ScratchRegister = temps.Acquire();
    ma_li(ScratchRegister, rt.immediate());
    divw(rd, rs, ScratchRegister);
  } else {
    MOZ_ASSERT(rt.is_reg());
    divw(rd, rs, rt.rm());
  }
}

void MacroAssemblerRiscv64::ma_divu32(Register rd, Register rs, Operand rt) {
  if (rt.is_imm()) {
    UseScratchRegisterScope temps(this);
    Register ScratchRegister = temps.Acquire();
    ma_li(ScratchRegister, rt.immediate());
    divuw(rd, rs, ScratchRegister);
  } else {
    MOZ_ASSERT(rt.is_reg());
    divuw(rd, rs, rt.rm());
  }
}

void MacroAssemblerRiscv64::ma_div64(Register rd, Register rs, Operand rt) {
  if (rt.is_imm()) {
    UseScratchRegisterScope temps(this);
    Register ScratchRegister = temps.Acquire();
    ma_li(ScratchRegister, rt.immediate());
    div(rd, rs, ScratchRegister);
  } else {
    MOZ_ASSERT(rt.is_reg());
    div(rd, rs, rt.rm());
  }
}

void MacroAssemblerRiscv64::ma_divu64(Register rd, Register rs, Operand rt) {
  if (rt.is_imm()) {
    UseScratchRegisterScope temps(this);
    Register ScratchRegister = temps.Acquire();
    ma_li(ScratchRegister, rt.immediate());
    divu(rd, rs, ScratchRegister);
  } else {
    MOZ_ASSERT(rt.is_reg());
    divu(rd, rs, rt.rm());
  }
}

void MacroAssemblerRiscv64::ma_mod32(Register rd, Register rs, Operand rt) {
  if (rt.is_imm()) {
    UseScratchRegisterScope temps(this);
    Register ScratchRegister = temps.Acquire();
    ma_li(ScratchRegister, rt.immediate());
    remw(rd, rs, ScratchRegister);
  } else {
    MOZ_ASSERT(rt.is_reg());
    remw(rd, rs, rt.rm());
  }
}

void MacroAssemblerRiscv64::ma_modu32(Register rd, Register rs, Operand rt) {
  if (rt.is_imm()) {
    UseScratchRegisterScope temps(this);
    Register ScratchRegister = temps.Acquire();
    ma_li(ScratchRegister, rt.immediate());
    remuw(rd, rs, ScratchRegister);
  } else {
    MOZ_ASSERT(rt.is_reg());
    remuw(rd, rs, rt.rm());
  }
}

void MacroAssemblerRiscv64::ma_mod64(Register rd, Register rs, Operand rt) {
  if (rt.is_imm()) {
    UseScratchRegisterScope temps(this);
    Register ScratchRegister = temps.Acquire();
    ma_li(ScratchRegister, rt.immediate());
    rem(rd, rs, ScratchRegister);
  } else {
    MOZ_ASSERT(rt.is_reg());
    rem(rd, rs, rt.rm());
  }
}

void MacroAssemblerRiscv64::ma_modu64(Register rd, Register rs, Operand rt) {
  if (rt.is_imm()) {
    UseScratchRegisterScope temps(this);
    Register ScratchRegister = temps.Acquire();
    ma_li(ScratchRegister, rt.immediate());
    remu(rd, rs, ScratchRegister);
  } else {
    MOZ_ASSERT(rt.is_reg());
    remu(rd, rs, rt.rm());
  }
}

void MacroAssemblerRiscv64::ma_mul32(Register rd, Register rs, Operand rt) {
  if (rt.is_imm()) {
    UseScratchRegisterScope temps(this);
    Register ScratchRegister = temps.Acquire();
    ma_li(ScratchRegister, rt.immediate());
    mulw(rd, rs, ScratchRegister);
  } else {
    MOZ_ASSERT(rt.is_reg());
    mulw(rd, rs, rt.rm());
  }
}

void MacroAssemblerRiscv64::ma_mulh32(Register rd, Register rs, Operand rt) {
  if (rt.is_imm()) {
    UseScratchRegisterScope temps(this);
    Register ScratchRegister = temps.Acquire();
    ma_li(ScratchRegister, rt.immediate());
    mul(rd, rs, ScratchRegister);
  } else {
    MOZ_ASSERT(rt.is_reg());
    mul(rd, rs, rt.rm());
  }
  srai(rd, rd, 32);
}

void MacroAssemblerRiscv64::ma_mul64(Register rd, Register rs, Operand rt) {
  if (rt.is_imm()) {
    UseScratchRegisterScope temps(this);
    Register ScratchRegister = temps.Acquire();
    ma_li(ScratchRegister, rt.immediate());
    mul(rd, rs, ScratchRegister);
  } else {
    MOZ_ASSERT(rt.is_reg());
    mul(rd, rs, rt.rm());
  }
}

void MacroAssemblerRiscv64::ma_mulh64(Register rd, Register rs, Operand rt) {
  if (rt.is_imm()) {
    UseScratchRegisterScope temps(this);
    Register ScratchRegister = temps.Acquire();
    ma_li(ScratchRegister, rt.immediate());
    mulh(rd, rs, ScratchRegister);
  } else {
    MOZ_ASSERT(rt.is_reg());
    mulh(rd, rs, rt.rm());
  }
}

void MacroAssemblerRiscv64::ma_sll64(Register rd, Register rs, Operand rt) {
  if (rt.is_reg()) {
    sll(rd, rs, rt.rm());
  } else {
    MOZ_ASSERT(rt.is_imm());
    uint8_t shamt = static_cast<uint8_t>(rt.immediate());
    slli(rd, rs, shamt);
  }
}

void MacroAssemblerRiscv64::ma_sll32(Register rd, Register rs, Operand rt) {
  if (rt.is_reg()) {
    sllw(rd, rs, rt.rm());
  } else {
    MOZ_ASSERT(rt.is_imm());
    uint8_t shamt = static_cast<uint8_t>(rt.immediate());
    slliw(rd, rs, shamt);
  }
}

void MacroAssemblerRiscv64::ma_sra64(Register rd, Register rs, Operand rt) {
  if (rt.is_reg()) {
    sra(rd, rs, rt.rm());
  } else {
    MOZ_ASSERT(rt.is_imm());
    uint8_t shamt = static_cast<uint8_t>(rt.immediate());
    srai(rd, rs, shamt);
  }
}

void MacroAssemblerRiscv64::ma_sra32(Register rd, Register rs, Operand rt) {
  if (rt.is_reg()) {
    sraw(rd, rs, rt.rm());
  } else {
    MOZ_ASSERT(rt.is_imm());
    uint8_t shamt = static_cast<uint8_t>(rt.immediate());
    sraiw(rd, rs, shamt);
  }
}

void MacroAssemblerRiscv64::ma_srl64(Register rd, Register rs, Operand rt) {
  if (rt.is_reg()) {
    srl(rd, rs, rt.rm());
  } else {
    MOZ_ASSERT(rt.is_imm());
    uint8_t shamt = static_cast<uint8_t>(rt.immediate());
    srli(rd, rs, shamt);
  }
}

void MacroAssemblerRiscv64::ma_srl32(Register rd, Register rs, Operand rt) {
  if (rt.is_reg()) {
    srlw(rd, rs, rt.rm());
  } else {
    MOZ_ASSERT(rt.is_imm());
    uint8_t shamt = static_cast<uint8_t>(rt.immediate());
    srliw(rd, rs, shamt);
  }
}

void MacroAssemblerRiscv64::ma_slt(Register rd, Register rs, Operand rt) {
  if (rt.is_reg()) {
    slt(rd, rs, rt.rm());
  } else {
    MOZ_ASSERT(rt.is_imm());
    if (is_int12(rt.immediate())) {
      slti(rd, rs, static_cast<int32_t>(rt.immediate()));
    } else {
      // li handles the relocation.
      UseScratchRegisterScope temps(this);
      Register scratch = temps.Acquire();
      BlockTrampolinePoolScope block_trampoline_pool(this);
      ma_li(scratch, rt.immediate());
      slt(rd, rs, scratch);
    }
  }
}

void MacroAssemblerRiscv64::ma_sltu(Register rd, Register rs, Operand rt) {
  if (rt.is_reg()) {
    sltu(rd, rs, rt.rm());
  } else {
    MOZ_ASSERT(rt.is_imm());
    if (is_int12(rt.immediate())) {
      sltiu(rd, rs, static_cast<int32_t>(rt.immediate()));
    } else {
      // li handles the relocation.
      UseScratchRegisterScope temps(this);
      Register scratch = temps.Acquire();
      BlockTrampolinePoolScope block_trampoline_pool(this);
      ma_li(scratch, rt.immediate());
      sltu(rd, rs, scratch);
    }
  }
}

void MacroAssemblerRiscv64::ma_sle(Register rd, Register rs, Operand rt) {
  if (rt.is_reg()) {
    slt(rd, rt.rm(), rs);
  } else {
    MOZ_ASSERT(rt.is_imm());
    // li handles the relocation.
    UseScratchRegisterScope temps(this);
    Register scratch = temps.Acquire();
    BlockTrampolinePoolScope block_trampoline_pool(this);
    ma_li(scratch, rt.immediate());
    slt(rd, scratch, rs);
  }
  xori(rd, rd, 1);
}

void MacroAssemblerRiscv64::ma_sleu(Register rd, Register rs, Operand rt) {
  if (rt.is_reg()) {
    sltu(rd, rt.rm(), rs);
  } else {
    MOZ_ASSERT(rt.is_imm());
    // li handles the relocation.
    UseScratchRegisterScope temps(this);
    Register scratch = temps.Acquire();
    BlockTrampolinePoolScope block_trampoline_pool(this);
    ma_li(scratch, rt.immediate());
    sltu(rd, scratch, rs);
  }
  xori(rd, rd, 1);
}

void MacroAssemblerRiscv64::ma_sgt(Register rd, Register rs, Operand rt) {
  if (rt.is_reg()) {
    slt(rd, rt.rm(), rs);
  } else {
    MOZ_ASSERT(rt.is_imm());
    // li handles the relocation.
    UseScratchRegisterScope temps(this);
    Register scratch = temps.Acquire();
    BlockTrampolinePoolScope block_trampoline_pool(this);
    ma_li(scratch, rt.immediate());
    slt(rd, scratch, rs);
  }
}

void MacroAssemblerRiscv64::ma_sgtu(Register rd, Register rs, Operand rt) {
  if (rt.is_reg()) {
    sltu(rd, rt.rm(), rs);
  } else {
    MOZ_ASSERT(rt.is_imm());
    // li handles the relocation.
    UseScratchRegisterScope temps(this);
    Register scratch = temps.Acquire();
    BlockTrampolinePoolScope block_trampoline_pool(this);
    ma_li(scratch, rt.immediate());
    sltu(rd, scratch, rs);
  }
}

void MacroAssemblerRiscv64::ma_sge(Register rd, Register rs, Operand rt) {
  ma_slt(rd, rs, rt);
  xori(rd, rd, 1);
}

void MacroAssemblerRiscv64::ma_sgeu(Register rd, Register rs, Operand rt) {
  ma_sltu(rd, rs, rt);
  xori(rd, rd, 1);
}

static inline bool IsZero(const Operand& rt) {
  if (rt.is_reg()) {
    return rt.rm() == zero_reg;
  } else {
    MOZ_ASSERT(rt.is_imm());
    return rt.immediate() == 0;
  }
}

void MacroAssemblerRiscv64::ma_seq(Register rd, Register rs, Operand rt) {
  if (rs == zero_reg) {
    ma_seqz(rd, rt);
  } else if (IsZero(rt)) {
    seqz(rd, rs);
  } else {
    ma_sub64(rd, rs, rt);
    seqz(rd, rd);
  }
}

void MacroAssemblerRiscv64::ma_sne(Register rd, Register rs, Operand rt) {
  if (rs == zero_reg) {
    ma_snez(rd, rt);
  } else if (IsZero(rt)) {
    snez(rd, rs);
  } else {
    ma_sub64(rd, rs, rt);
    snez(rd, rd);
  }
}

void MacroAssemblerRiscv64::ma_seqz(Register rd, const Operand& rt) {
  if (rt.is_reg()) {
    seqz(rd, rt.rm());
  } else {
    ma_li(rd, rt.immediate() == 0);
  }
}

void MacroAssemblerRiscv64::ma_snez(Register rd, const Operand& rt) {
  if (rt.is_reg()) {
    snez(rd, rt.rm());
  } else {
    ma_li(rd, rt.immediate() != 0);
  }
}

void MacroAssemblerRiscv64::ma_neg(Register rd, const Operand& rt) {
  MOZ_ASSERT(rt.is_reg());
  neg(rd, rt.rm());
}

void MacroAssemblerRiscv64::ma_jump(ImmPtr dest) {
  UseScratchRegisterScope temps(this);
  Register scratch = temps.Acquire();
  asMasm().ma_liPatchable(scratch, dest);
  jr(scratch, 0);
}
// fp instructions
void MacroAssemblerRiscv64::ma_lid(FloatRegister dest, double value) {
  ImmWord imm(mozilla::BitwiseCast<uint64_t>(value));

  if (imm.value != 0) {
    UseScratchRegisterScope temps(this);
    Register scratch = temps.Acquire();
    ma_li(scratch, imm);
    fmv_d_x(dest, scratch);
  } else {
    fmv_d_x(dest, zero);
  }
}
// fp instructions
void MacroAssemblerRiscv64::ma_lis(FloatRegister dest, float value) {
  Imm32 imm(mozilla::BitwiseCast<uint32_t>(value));

  if (imm.value != 0) {
    UseScratchRegisterScope temps(this);
    Register scratch = temps.Acquire();
    ma_li(scratch, imm);
    fmv_w_x(dest, scratch);
  } else {
    fmv_w_x(dest, zero);
  }
}

void MacroAssemblerRiscv64::ma_sub32TestOverflow(Register rd,
                                                 Register rj,
                                                 Register rk,
                                                 Label* overflow) {
  UseScratchRegisterScope temps(this);
  Register scratch = temps.Acquire();
  sub(scratch, rj, rk);
  subw(rd, rj, rk);
  ma_b(rd, Register(scratch), overflow, Assembler::NotEqual);
}

void MacroAssemblerRiscv64::ma_sub32TestOverflow(Register rd,
                                                 Register rj,
                                                 Imm32 imm,
                                                 Label* overflow) {
  if (imm.value != INT32_MIN) {
    asMasm().ma_add32TestOverflow(rd, rj, Imm32(-imm.value), overflow);
  } else {
    UseScratchRegisterScope temps(this);
    Register scratch = temps.Acquire();
    MOZ_ASSERT(rj != scratch);
    ma_li(scratch, Imm32(imm.value));
    asMasm().ma_sub32TestOverflow(rd, rj, scratch, overflow);
  }
}

void MacroAssemblerRiscv64::ma_add32TestOverflow(Register rd,
                                                 Register rj,
                                                 Register rk,
                                                 Label* overflow) {
  UseScratchRegisterScope temps(this);
  Register scratch = temps.Acquire();
  add(scratch, rj, rk);
  addw(rd, rj, rk);
  ma_b(rd, Register(scratch), overflow, Assembler::NotEqual);
}

void MacroAssemblerRiscv64::ma_add32TestOverflow(Register rd,
                                                 Register rj,
                                                 Imm32 imm,
                                                 Label* overflow) {
  // Check for signed range because of addi
  if (is_intn(imm.value, 12)) {
    UseScratchRegisterScope temps(this);
    Register scratch = temps.Acquire();
    addi(scratch, rj, imm.value);
    addiw(rd, rj, imm.value);
    ma_b(rd, scratch, overflow, Assembler::NotEqual);
  } else {
    UseScratchRegisterScope temps(this);
    Register scratch2 = temps.Acquire();
    ma_li(scratch2, imm);
    ma_add32TestOverflow(rd, rj, scratch2, overflow);
  }
}

void MacroAssemblerRiscv64::ma_subPtrTestOverflow(Register rd,
                                                  Register rj,
                                                  Register rk,
                                                  Label* overflow) {
  UseScratchRegisterScope temps(this);
  Register scratch2 = temps.Acquire();
  MOZ_ASSERT_IF(rj == rd, rj != rk);
  MOZ_ASSERT(rj != scratch2);
  MOZ_ASSERT(rk != scratch2);
  MOZ_ASSERT(rd != scratch2);

  Register rj_copy = rj;

  if (rj == rd) {
    ma_or(scratch2, rj, zero);
    rj_copy = scratch2;
  }

  {
    Register scratch = temps.Acquire();
    MOZ_ASSERT(rd != scratch);

    sub(rd, rj, rk);
    // If the sign of rj and rk are the same, no overflow
    ma_xor(scratch, rj_copy, rk);
    // Check if the sign of rd and rj are the same
    ma_xor(scratch2, rd, rj_copy);
    ma_and(scratch2, scratch2, scratch);
  }

  ma_b(scratch2, zero, overflow, Assembler::LessThan);
}

void MacroAssemblerRiscv64::ma_addPtrTestOverflow(Register rd,
                                                  Register rj,
                                                  Register rk,
                                                  Label* overflow) {
  UseScratchRegisterScope temps(this);
  Register scratch = temps.Acquire();
  MOZ_ASSERT(rd != scratch);

  if (rj == rk) {
    if (rj == rd) {
      ma_or(scratch, rj, zero);
      rj = scratch;
    }

    add(rd, rj, rj);
    ma_xor(scratch, rj, rd);
    ma_b(scratch, zero, overflow, Assembler::LessThan);
  } else {
    UseScratchRegisterScope temps(this);
    Register scratch2 = temps.Acquire();
    MOZ_ASSERT(rj != scratch);
    MOZ_ASSERT(rd != scratch2);

    if (rj == rd) {
      ma_or(scratch2, rj, zero);
      rj = scratch2;
    }

    add(rd, rj, rk);
    slti(scratch, rj, 0);
    slt(scratch2, rd, rj);
    ma_b(scratch, Register(scratch2), overflow, Assembler::NotEqual);
  }
}

void MacroAssemblerRiscv64::ma_addPtrTestOverflow(Register rd,
                                                  Register rj,
                                                  Imm32 imm,
                                                  Label* overflow) {
  UseScratchRegisterScope temps(this);
  Register scratch2 = temps.Acquire();

  if (imm.value == 0) {
    ori(rd, rj, 0);
    return;
  }

  if (rj == rd) {
    ori(scratch2, rj, 0);
    rj = scratch2;
  }

  ma_add64(rd, rj, imm);

  if (imm.value > 0) {
    ma_b(rd, rj, overflow, Assembler::LessThan);
  } else {
    MOZ_ASSERT(imm.value < 0);
    ma_b(rd, rj, overflow, Assembler::GreaterThan);
  }
}

void MacroAssemblerRiscv64::ma_add32TestCarry(Condition cond,
                                              Register rd,
                                              Register rj,
                                              Register rk,
                                              Label* overflow) {
  MOZ_ASSERT(cond == Assembler::CarrySet || cond == Assembler::CarryClear);
  MOZ_ASSERT_IF(rd == rj, rk != rd);
  UseScratchRegisterScope temps(this);
  Register scratch = temps.Acquire();
  addw(rd, rj, rk);
  sltu(scratch, rd, rd == rj ? rk : rj);
  ma_b(Register(scratch), Register(scratch), overflow,
       cond == Assembler::CarrySet ? Assembler::NonZero : Assembler::Zero);
}

void MacroAssemblerRiscv64::ma_add32TestCarry(Condition cond,
                                              Register rd,
                                              Register rj,
                                              Imm32 imm,
                                              Label* overflow) {
  UseScratchRegisterScope temps(this);
  Register scratch2 = temps.Acquire();
  MOZ_ASSERT(rj != scratch2);
  ma_li(scratch2, imm);
  ma_add32TestCarry(cond, rd, rj, scratch2, overflow);
}

void MacroAssemblerRiscv64::ma_subPtrTestOverflow(Register rd,
                                                  Register rj,
                                                  Imm32 imm,
                                                  Label* overflow) {
  // TODO(loong64): Check subPtrTestOverflow
  MOZ_ASSERT(imm.value != INT32_MIN);
  ma_addPtrTestOverflow(rd, rj, Imm32(-imm.value), overflow);
}

void MacroAssemblerRiscv64::ma_addPtrTestCarry(Condition cond,
                                               Register rd,
                                               Register rj,
                                               Register rk,
                                               Label* label) {
  UseScratchRegisterScope temps(this);
  Register scratch = temps.Acquire();
  MOZ_ASSERT(rd != rk);
  MOZ_ASSERT(rd != scratch);
  add(rd, rj, rk);
  sltu(scratch, rd, rk);
  ma_b(scratch, Register(scratch), label,
       cond == Assembler::CarrySet ? Assembler::NonZero : Assembler::Zero);
}

void MacroAssemblerRiscv64::ma_addPtrTestCarry(Condition cond,
                                               Register rd,
                                               Register rj,
                                               Imm32 imm,
                                               Label* label) {
  UseScratchRegisterScope temps(this);
  Register scratch2 = temps.Acquire();

  // Check for signed range because of addi
  if (is_intn(imm.value, 12)) {
    addi(rd, rj, imm.value);
    sltiu(scratch2, rd, imm.value);
    ma_b(scratch2, scratch2, label,
         cond == Assembler::CarrySet ? Assembler::NonZero : Assembler::Zero);
  } else {
    ma_li(scratch2, imm);
    ma_addPtrTestCarry(cond, rd, rj, scratch2, label);
  }
}

void MacroAssemblerRiscv64::ma_addPtrTestCarry(Condition cond,
                                               Register rd,
                                               Register rj,
                                               ImmWord imm,
                                               Label* label) {
  UseScratchRegisterScope temps(this);
  Register scratch2 = temps.Acquire();

  // Check for signed range because of as_addi_d
  if (is_intn(imm.value, 12)) {
    uint32_t value = imm.value;
    addi(rd, rj, value);
    ma_sltu(scratch2, rd, Operand(value));
    ma_b(scratch2, scratch2, label,
         cond == Assembler::CarrySet ? Assembler::NonZero : Assembler::Zero);
  } else {
    ma_li(scratch2, imm);
    ma_addPtrTestCarry(cond, rd, rj, scratch2, label);
  }
}

void MacroAssemblerRiscv64::ma_load(Register dest,
                                    const BaseIndex& src,
                                    LoadStoreSize size,
                                    LoadStoreExtension extension) {
  UseScratchRegisterScope temps(this);
  Register scratch2 = temps.Acquire();
  asMasm().computeScaledAddress(src, scratch2);
  asMasm().ma_load(dest, Address(scratch2, src.offset), size, extension);
}
void MacroAssemblerRiscv64::ma_pop(FloatRegister f) {
  fld(f, StackPointer, 0);
  addi(StackPointer, StackPointer, sizeof(double));
}

void MacroAssemblerRiscv64::ma_push(FloatRegister f) {
  addi(StackPointer, StackPointer, (int32_t) - sizeof(double));
  fsd(f, StackPointer, 0);
}

void MacroAssemblerRiscv64::ma_fld_s(FloatRegister ft, Address address) {
  int32_t offset = address.offset;
  Register base = address.base;

  if (is_intn(offset, 12)) {
    flw(ft, base, offset);
  } else {
    UseScratchRegisterScope temps(this);
    Register scratch = temps.Acquire();
    MOZ_ASSERT(base != scratch);
    ma_li(scratch, Imm32(offset));
    ma_add64(scratch, base, scratch);
    flw(ft, scratch, 0);
  }
}
void MacroAssemblerRiscv64::ma_fld_d(FloatRegister ft, Address address) {
  int32_t offset = address.offset;
  Register base = address.base;

  if (is_intn(offset, 12)) {
    fld(ft, base, offset);
  } else {
    UseScratchRegisterScope temps(this);
    Register scratch = temps.Acquire();
    MOZ_ASSERT(base != scratch);
    ma_li(scratch, Imm32(offset));
    ma_add64(scratch, base, scratch);
    fld(ft, scratch, 0);
  }
}
void MacroAssemblerRiscv64::ma_fst_d(FloatRegister ft, Address address) {
  int32_t offset = address.offset;
  Register base = address.base;

  if (is_intn(offset, 12)) {
    fsd(ft, base, offset);
  } else {
    UseScratchRegisterScope temps(this);
    Register scratch = temps.Acquire();
    MOZ_ASSERT(base != scratch);
    ma_li(scratch, Imm32(offset));
    ma_add64(scratch, base, scratch);
    fsd(ft, scratch, 0);
  }
}
void MacroAssemblerRiscv64::ma_fst_s(FloatRegister ft, Address address) {
  int32_t offset = address.offset;
  Register base = address.base;

  if (is_intn(offset, 12)) {
    fsw(ft, base, offset);
  } else {
    UseScratchRegisterScope temps(this);
    Register scratch = temps.Acquire();
    MOZ_ASSERT(base != scratch);
    ma_li(scratch, Imm32(offset));
    ma_add64(scratch, base, scratch);
    fsw(ft, scratch, 0);
  }
}

void MacroAssemblerRiscv64::ma_fst_d(FloatRegister ft, BaseIndex address) {
  UseScratchRegisterScope temps(this);
  Register scratch = temps.Acquire();
  asMasm().computeScaledAddress(address, scratch);
  asMasm().ma_fst_d(ft, Address(scratch, address.offset));
}

void MacroAssemblerRiscv64::ma_fst_s(FloatRegister ft, BaseIndex address) {
  UseScratchRegisterScope temps(this);
  Register scratch = temps.Acquire();
  asMasm().computeScaledAddress(address, scratch);
  asMasm().ma_fst_s(ft, Address(scratch, address.offset));
}

void MacroAssemblerRiscv64::ma_fld_d(FloatRegister ft, const BaseIndex& src) {
  UseScratchRegisterScope temps(this);
  Register scratch = temps.Acquire();
  asMasm().computeScaledAddress(src, scratch);
  asMasm().ma_fld_d(ft, Address(scratch, src.offset));
}

void MacroAssemblerRiscv64::ma_fld_s(FloatRegister ft, const BaseIndex& src) {
  UseScratchRegisterScope temps(this);
  Register scratch = temps.Acquire();
  asMasm().computeScaledAddress(src, scratch);
  asMasm().ma_fld_s(ft, Address(scratch, src.offset));
}

void MacroAssemblerRiscv64::ma_call(ImmPtr dest) {
  asMasm().ma_liPatchable(CallReg, dest);
  jalr(CallReg, 0);
}

void MacroAssemblerRiscv64::CompareIsNotNanF32(Register rd,
                                               FPURegister cmp1,
                                               FPURegister cmp2) {
  UseScratchRegisterScope temps(this);
  BlockTrampolinePoolScope block_trampoline_pool(this);
  Register scratch = temps.Acquire();

  feq_s(rd, cmp1, cmp1);       // rd <- !isNan(cmp1)
  feq_s(scratch, cmp2, cmp2);  // scratch <- !isNaN(cmp2)
  ma_and(rd, rd, scratch);     // rd <- !isNan(cmp1) && !isNan(cmp2)
}

void MacroAssemblerRiscv64::CompareIsNotNanF64(Register rd,
                                               FPURegister cmp1,
                                               FPURegister cmp2) {
  UseScratchRegisterScope temps(this);
  BlockTrampolinePoolScope block_trampoline_pool(this);
  Register scratch = temps.Acquire();

  feq_d(rd, cmp1, cmp1);       // rd <- !isNan(cmp1)
  feq_d(scratch, cmp2, cmp2);  // scratch <- !isNaN(cmp2)
  ma_and(rd, rd, scratch);     // rd <- !isNan(cmp1) && !isNan(cmp2)
}

void MacroAssemblerRiscv64::CompareIsNanF32(Register rd,
                                            FPURegister cmp1,
                                            FPURegister cmp2) {
  CompareIsNotNanF32(rd, cmp1, cmp2);  // rd <- !isNan(cmp1) && !isNan(cmp2)
  ma_xor(rd, rd, Operand(1));          // rd <- isNan(cmp1) || isNan(cmp2)
}

void MacroAssemblerRiscv64::CompareIsNanF64(Register rd,
                                            FPURegister cmp1,
                                            FPURegister cmp2) {
  CompareIsNotNanF64(rd, cmp1, cmp2);  // rd <- !isNan(cmp1) && !isNan(cmp2)
  ma_xor(rd, rd, Operand(1));          // rd <- isNan(cmp1) || isNan(cmp2)
}

void MacroAssemblerRiscv64::Clz32(Register rd, Register xx) {
  // 32 bit unsigned in lower word: count number of leading zeros.
  //  int n = 32;
  //  unsigned y;

  //  y = x >>16; if (y != 0) { n = n -16; x = y; }
  //  y = x >> 8; if (y != 0) { n = n - 8; x = y; }
  //  y = x >> 4; if (y != 0) { n = n - 4; x = y; }
  //  y = x >> 2; if (y != 0) { n = n - 2; x = y; }
  //  y = x >> 1; if (y != 0) {rd = n - 2; return;}
  //  rd = n - x;

  Label L0, L1, L2, L3, L4;
  UseScratchRegisterScope temps(this);
  BlockTrampolinePoolScope block_trampoline_pool(this);
  Register x = rd;
  Register y = temps.Acquire();
  Register n = temps.Acquire();
  MOZ_ASSERT(xx != y && xx != n);
  mv(x, xx);
  ma_li(n, Imm32(32));
#if JS_CODEGEN_RISCV64
  srliw(y, x, 16);
  ma_branch(&L0, Equal, y, Operand(zero_reg));
  mv(x, y);
  addiw(n, n, -16);
  bind(&L0);
  srliw(y, x, 8);
  ma_branch(&L1, Equal, y, Operand(zero_reg));
  addiw(n, n, -8);
  mv(x, y);
  bind(&L1);
  srliw(y, x, 4);
  ma_branch(&L2, Equal, y, Operand(zero_reg));
  addiw(n, n, -4);
  mv(x, y);
  bind(&L2);
  srliw(y, x, 2);
  ma_branch(&L3, Equal, y, Operand(zero_reg));
  addiw(n, n, -2);
  mv(x, y);
  bind(&L3);
  srliw(y, x, 1);
  subw(rd, n, x);
  ma_branch(&L4, Equal, y, Operand(zero_reg));
  addiw(rd, n, -2);
  bind(&L4);
#elif JS_CODEGEN_RISCV32
  srli(y, x, 16);
  ma_branch(&L0, Equal, y, Operand(zero_reg));
  mv(x, y);
  addi(n, n, -16);
  bind(&L0);
  srli(y, x, 8);
  ma_branch(&L1, Equal, y, Operand(zero_reg));
  addi(n, n, -8);
  mv(x, y);
  bind(&L1);
  srli(y, x, 4);
  ma_branch(&L2, Equal, y, Operand(zero_reg));
  addi(n, n, -4);
  mv(x, y);
  bind(&L2);
  srli(y, x, 2);
  ma_branch(&L3, Equal, y, Operand(zero_reg));
  addi(n, n, -2);
  mv(x, y);
  bind(&L3);
  srli(y, x, 1);
  sub(rd, n, x);
  ma_branch(&L4, Equal, y, Operand(zero_reg));
  addi(rd, n, -2);
  bind(&L4);
#endif
}

#if JS_CODEGEN_RISCV64
void MacroAssemblerRiscv64::Clz64(Register rd, Register xx) {
  // 64 bit: count number of leading zeros.
  //  int n = 64;
  //  unsigned y;

  //  y = x >>32; if (y != 0) { n = n - 32; x = y; }
  //  y = x >>16; if (y != 0) { n = n - 16; x = y; }
  //  y = x >> 8; if (y != 0) { n = n - 8; x = y; }
  //  y = x >> 4; if (y != 0) { n = n - 4; x = y; }
  //  y = x >> 2; if (y != 0) { n = n - 2; x = y; }
  //  y = x >> 1; if (y != 0) {rd = n - 2; return;}
  //  rd = n - x;

  Label L0, L1, L2, L3, L4, L5;
  UseScratchRegisterScope temps(this);
  BlockTrampolinePoolScope block_trampoline_pool(this);
  Register x = rd;
  Register y = temps.Acquire();
  Register n = temps.Acquire();
  MOZ_ASSERT(xx != y && xx != n);
  mv(x, xx);
  ma_li(n, Imm32(64));
  srli(y, x, 32);
  ma_branch(&L0, Equal, y, Operand(zero_reg));
  addiw(n, n, -32);
  mv(x, y);
  bind(&L0);
  srli(y, x, 16);
  ma_branch(&L1, Equal, y, Operand(zero_reg));
  addiw(n, n, -16);
  mv(x, y);
  bind(&L1);
  srli(y, x, 8);
  ma_branch(&L2, Equal, y, Operand(zero_reg));
  addiw(n, n, -8);
  mv(x, y);
  bind(&L2);
  srli(y, x, 4);
  ma_branch(&L3, Equal, y, Operand(zero_reg));
  addiw(n, n, -4);
  mv(x, y);
  bind(&L3);
  srli(y, x, 2);
  ma_branch(&L4, Equal, y, Operand(zero_reg));
  addiw(n, n, -2);
  mv(x, y);
  bind(&L4);
  srli(y, x, 1);
  subw(rd, n, x);
  ma_branch(&L5, Equal, y, Operand(zero_reg));
  addiw(rd, n, -2);
  bind(&L5);
}
#endif
void MacroAssemblerRiscv64::Ctz32(Register rd, Register rs) {
  // Convert trailing zeroes to trailing ones, and bits to their left
  // to zeroes.

  BlockTrampolinePoolScope block_trampoline_pool(this);
  {
    UseScratchRegisterScope temps(this);
    Register scratch = temps.Acquire();
    ma_add64(scratch, rs, Operand(-1));
    ma_xor(rd, scratch, rs);
    ma_and(rd, rd, scratch);
    // Count number of leading zeroes.
  }
  Clz32(rd, rd);
  {
    // Subtract number of leading zeroes from 32 to get number of trailing
    // ones. Remember that the trailing ones were formerly trailing zeroes.
    UseScratchRegisterScope temps(this);
    Register scratch = temps.Acquire();
    ma_li(scratch, Imm32(32));
    ma_sub32(rd, scratch, rd);
  }
}
#if JS_CODEGEN_RISCV64
void MacroAssemblerRiscv64::Ctz64(Register rd, Register rs) {
  // Convert trailing zeroes to trailing ones, and bits to their left
  // to zeroes.
  BlockTrampolinePoolScope block_trampoline_pool(this);
  {
    UseScratchRegisterScope temps(this);
    Register scratch = temps.Acquire();
    ma_add64(scratch, rs, Operand(-1));
    ma_xor(rd, scratch, rs);
    ma_and(rd, rd, scratch);
    // Count number of leading zeroes.
  }
  Clz64(rd, rd);
  {
    // Subtract number of leading zeroes from 64 to get number of trailing
    // ones. Remember that the trailing ones were formerly trailing zeroes.
    UseScratchRegisterScope temps(this);
    Register scratch = temps.Acquire();
    ma_li(scratch, 64);
    ma_sub64(rd, scratch, rd);
  }
}
#endif
void MacroAssemblerRiscv64::Popcnt32(Register rd,
                                     Register rs,
                                     Register scratch) {
  MOZ_ASSERT(scratch != rs);
  MOZ_ASSERT(scratch != rd);
  // https://graphics.stanford.edu/~seander/bithacks.html#CountBitsSetParallel
  //
  // A generalization of the best bit counting method to integers of
  // bit-widths up to 128 (parameterized by type T) is this:
  //
  // v = v - ((v >> 1) & (T)~(T)0/3);                           // temp
  // v = (v & (T)~(T)0/15*3) + ((v >> 2) & (T)~(T)0/15*3);      // temp
  // v = (v + (v >> 4)) & (T)~(T)0/255*15;                      // temp
  // c = (T)(v * ((T)~(T)0/255)) >> (sizeof(T) - 1) * BITS_PER_BYTE; //count
  //
  // There are algorithms which are faster in the cases where very few
  // bits are set but the algorithm here attempts to minimize the total
  // number of instructions executed even when a large number of bits
  // are set.
  // The number of instruction is 20.
  // uint32_t B0 = 0x55555555;     // (T)~(T)0/3
  // uint32_t B1 = 0x33333333;     // (T)~(T)0/15*3
  // uint32_t B2 = 0x0F0F0F0F;     // (T)~(T)0/255*15
  // uint32_t value = 0x01010101;  // (T)~(T)0/255

  uint32_t shift = 24;
  UseScratchRegisterScope temps(this);
  BlockTrampolinePoolScope block_trampoline_pool(this);
  Register scratch2 = temps.Acquire();
  Register value = temps.Acquire();
  MOZ_ASSERT((rd != value) && (rs != value));
  ma_li(value, 0x01010101);     // value = 0x01010101;
  ma_li(scratch2, 0x55555555);  // B0 = 0x55555555;
  ma_srl32(scratch, rs, Operand(1));
  ma_and(scratch, scratch, scratch2);
  ma_sub32(scratch, rs, scratch);
  ma_li(scratch2, 0x33333333);  // B1 = 0x33333333;
  slli(rd, scratch2, 4);
  or_(scratch2, scratch2, rd);
  ma_and(rd, scratch, scratch2);
  ma_srl32(scratch, scratch, Operand(2));
  ma_and(scratch, scratch, scratch2);
  ma_add32(scratch, rd, scratch);
  ma_srl32(rd, scratch, Operand(4));
  ma_add32(rd, rd, scratch);
  ma_li(scratch2, 0xF);
  ma_mul32(scratch2, value, scratch2);  // B2 = 0x0F0F0F0F;
  ma_and(rd, rd, scratch2);
  ma_mul32(rd, rd, value);
  ma_srl32(rd, rd, Operand(shift));
}

#if JS_CODEGEN_RISCV64
void MacroAssemblerRiscv64::Popcnt64(Register rd,
                                     Register rs,
                                     Register scratch) {
  MOZ_ASSERT(scratch != rs);
  MOZ_ASSERT(scratch != rd);
  // uint64_t B0 = 0x5555555555555555l;     // (T)~(T)0/3
  // uint64_t B1 = 0x3333333333333333l;     // (T)~(T)0/15*3
  // uint64_t B2 = 0x0F0F0F0F0F0F0F0Fl;     // (T)~(T)0/255*15
  // uint64_t value = 0x0101010101010101l;  // (T)~(T)0/255
  // uint64_t shift = 24;                   // (sizeof(T) - 1) * BITS_PER_BYTE
  uint64_t shift = 24;
  UseScratchRegisterScope temps(this);
  BlockTrampolinePoolScope block_trampoline_pool(this);
  Register scratch2 = temps.Acquire();
  Register value = temps.Acquire();
  MOZ_ASSERT((rd != value) && (rs != value));
  ma_li(value, 0x1111111111111111l);  // value = 0x1111111111111111l;
  ma_li(scratch2, 5);
  ma_mul64(scratch2, value, scratch2);  // B0 = 0x5555555555555555l;
  ma_srl64(scratch, rs, Operand(1));
  ma_and(scratch, scratch, scratch2);
  ma_sub64(scratch, rs, scratch);
  ma_li(scratch2, 3);
  ma_mul64(scratch2, value, scratch2);  // B1 = 0x3333333333333333l;
  ma_and(rd, scratch, scratch2);
  ma_srl64(scratch, scratch, Operand(2));
  ma_and(scratch, scratch, scratch2);
  ma_add64(scratch, rd, scratch);
  ma_srl64(rd, scratch, Operand(4));
  ma_add64(rd, rd, scratch);
  ma_li(scratch2, 0xF);
  ma_li(value, 0x0101010101010101l);    // value = 0x0101010101010101l;
  ma_mul64(scratch2, value, scratch2);  // B2 = 0x0F0F0F0F0F0F0F0Fl;
  ma_and(rd, rd, scratch2);
  ma_mul64(rd, rd, value);
  srli(rd, rd, 32 + shift);
}
#endif

void MacroAssemblerRiscv64::ma_div_branch_overflow(Register rd,
                                                   Register rj,
                                                   Register rk,
                                                   Label* overflow) {
  ScratchRegisterScope scratch(asMasm());
  ma_mod32(scratch, rj, rk);
  ma_b(scratch, scratch, overflow, Assembler::NonZero);
  divw(rd, rj, rk);
}

void MacroAssemblerRiscv64::ma_div_branch_overflow(Register rd,
                                                   Register rj,
                                                   Imm32 imm,
                                                   Label* overflow) {
  UseScratchRegisterScope temps(this);
  Register scratch = temps.Acquire();
  ma_li(scratch, imm);
  ma_div_branch_overflow(rd, rj, scratch, overflow);
}

void MacroAssemblerRiscv64::ma_mod_mask(Register src,
                                        Register dest,
                                        Register hold,
                                        Register remain,
                                        int32_t shift,
                                        Label* negZero) {
  // MATH:
  // We wish to compute x % (1<<y) - 1 for a known constant, y.
  // First, let b = (1<<y) and C = (1<<y)-1, then think of the 32 bit
  // dividend as a number in base b, namely
  // c_0*1 + c_1*b + c_2*b^2 ... c_n*b^n
  // now, since both addition and multiplication commute with modulus,
  // x % C == (c_0 + c_1*b + ... + c_n*b^n) % C ==
  // (c_0 % C) + (c_1%C) * (b % C) + (c_2 % C) * (b^2 % C)...
  // now, since b == C + 1, b % C == 1, and b^n % C == 1
  // this means that the whole thing simplifies to:
  // c_0 + c_1 + c_2 ... c_n % C
  // each c_n can easily be computed by a shift/bitextract, and the modulus
  // can be maintained by simply subtracting by C whenever the number gets
  // over C.
  int32_t mask = (1 << shift) - 1;
  Label head, negative, sumSigned, done;

  // hold holds -1 if the value was negative, 1 otherwise.
  // remain holds the remaining bits that have not been processed
  // SecondScratchReg serves as a temporary location to store extracted bits
  // into as well as holding the trial subtraction as a temp value dest is
  // the accumulator (and holds the final result)

  // move the whole value into the remain.
  or_(remain, src, zero);
  // Zero out the dest.
  ma_li(dest, Imm32(0));
  // Set the hold appropriately.
  ma_b(remain, remain, &negative, Signed, ShortJump);
  ma_li(hold, Imm32(1));
  ma_branch(&head, ShortJump);

  bind(&negative);
  ma_li(hold, Imm32(-1));
  subw(remain, zero, remain);

  // Begin the main loop.
  bind(&head);

  UseScratchRegisterScope temps(this);
  Register scratch2 = temps.Acquire();
  // Extract the bottom bits into SecondScratchReg.
  ma_and(scratch2, remain, Imm32(mask));
  // Add those bits to the accumulator.
  addw(dest, dest, scratch2);
  // Do a trial subtraction
  ma_sub32(scratch2, dest, Imm32(mask));
  // If (sum - C) > 0, store sum - C back into sum, thus performing a
  // modulus.
  ma_b(scratch2, Register(scratch2), &sumSigned, Signed, ShortJump);
  or_(dest, scratch2, zero);
  bind(&sumSigned);
  // Get rid of the bits that we extracted before.
  srliw(remain, remain, shift);
  // If the shift produced zero, finish, otherwise, continue in the loop.
  ma_b(remain, remain, &head, NonZero, ShortJump);
  // Check the hold to see if we need to negate the result.
  ma_b(hold, hold, &done, NotSigned, ShortJump);

  // If the hold was non-zero, negate the result to be in line with
  // what JS wants
  if (negZero != nullptr) {
    // Jump out in case of negative zero.
    ma_b(hold, hold, negZero, Zero);
    subw(dest, zero, dest);
  } else {
    subw(dest, zero, dest);
  }

  bind(&done);
}

void MacroAssemblerRiscv64::ma_fmovz(FloatFormat fmt,
                                     FloatRegister fd,
                                     FloatRegister fj,
                                     Register rk) {
  Label done;
  ma_b(rk, zero, &done, Assembler::NotEqual);
  if (fmt == SingleFloat) {
    fmv_s(fd, fj);
  } else {
    fmv_d(fd, fj);
  }
  bind(&done);
}

void MacroAssemblerRiscv64::ByteSwap(Register rd, Register rs, int operand_size,
                              Register scratch) {
  MOZ_ASSERT(scratch != rs);
  MOZ_ASSERT(scratch != rd);
  MOZ_ASSERT(operand_size == 4 || operand_size == 8);
  if (operand_size == 4) {
    // Uint32_t x1 = 0x00FF00FF;
    // x0 = (x0 << 16 | x0 >> 16);
    // x0 = (((x0 & x1) << 8)  | ((x0 & (x1 << 8)) >> 8));
    UseScratchRegisterScope temps(this);
    BlockTrampolinePoolScope block_trampoline_pool(this);
    MOZ_ASSERT((rd != t6) && (rs != t6));
    Register x0 = temps.Acquire();
    Register x1 = temps.Acquire();
    Register x2 = scratch;
    RV_li(x1, 0x00FF00FF);
    slliw(x0, rs, 16);
    srliw(rd, rs, 16);
    or_(x0, rd, x0);   // x0 <- x0 << 16 | x0 >> 16
    and_(x2, x0, x1);  // x2 <- x0 & 0x00FF00FF
    slliw(x2, x2, 8);  // x2 <- (x0 & x1) << 8
    slliw(x1, x1, 8);  // x1 <- 0xFF00FF00
    and_(rd, x0, x1);  // x0 & 0xFF00FF00
    srliw(rd, rd, 8);
    or_(rd, rd, x2);  // (((x0 & x1) << 8)  | ((x0 & (x1 << 8)) >> 8))
  } else {
    // uinx24_t x1 = 0x0000FFFF0000FFFFl;
    // uinx24_t x1 = 0x00FF00FF00FF00FFl;
    // x0 = (x0 << 32 | x0 >> 32);
    // x0 = (x0 & x1) << 16 | (x0 & (x1 << 16)) >> 16;
    // x0 = (x0 & x1) << 8  | (x0 & (x1 << 8)) >> 8;
    UseScratchRegisterScope temps(this);
    BlockTrampolinePoolScope block_trampoline_pool(this);
    MOZ_ASSERT((rd != t6) && (rs != t6));
    Register x0 = temps.Acquire();
    Register x1 = temps.Acquire();
    Register x2 = scratch;
    RV_li(x1, 0x0000FFFF0000FFFFl);
    slli(x0, rs, 32);
    srli(rd, rs, 32);
    or_(x0, rd, x0);   // x0 <- x0 << 32 | x0 >> 32
    and_(x2, x0, x1);  // x2 <- x0 & 0x0000FFFF0000FFFF
    slli(x2, x2, 16);  // x2 <- (x0 & 0x0000FFFF0000FFFF) << 16
    slli(x1, x1, 16);  // x1 <- 0xFFFF0000FFFF0000
    and_(rd, x0, x1);  // rd <- x0 & 0xFFFF0000FFFF0000
    srli(rd, rd, 16);  // rd <- x0 & (x1 << 16)) >> 16
    or_(x0, rd, x2);   // (x0 & x1) << 16 | (x0 & (x1 << 16)) >> 16;
    RV_li(x1, 0x00FF00FF00FF00FFl);
    and_(x2, x0, x1);  // x2 <- x0 & 0x00FF00FF00FF00FF
    slli(x2, x2, 8);   // x2 <- (x0 & x1) << 8
    slli(x1, x1, 8);   // x1 <- 0xFF00FF00FF00FF00
    and_(rd, x0, x1);
    srli(rd, rd, 8);  // rd <- (x0 & (x1 << 8)) >> 8
    or_(rd, rd, x2);  // (((x0 & x1) << 8)  | ((x0 & (x1 << 8)) >> 8))
  }
}

template <typename F_TYPE>
void MacroAssemblerRiscv64::FloatMinMaxHelper(FPURegister dst, FPURegister src1,
                                       FPURegister src2, MaxMinKind kind) {
  MOZ_ASSERT((std::is_same<F_TYPE, float>::value) ||
         (std::is_same<F_TYPE, double>::value));

  if (src1 == src2 && dst != src1) {
    if (std::is_same<float, F_TYPE>::value) {
      fmv_s(dst, src1);
    } else {
      fmv_d(dst, src1);
    }
    return;
  }

  Label done, nan;

  // For RISCV, fmin_s returns the other non-NaN operand as result if only one
  // operand is NaN; but for JS, if any operand is NaN, result is Nan. The
  // following handles the discrepency between handling of NaN between ISA and
  // JS semantics
  UseScratchRegisterScope temps(this);
  Register scratch = temps.Acquire();
  if (std::is_same<float, F_TYPE>::value) {
    CompareIsNotNanF32(scratch, src1, src2);
  } else {
    CompareIsNotNanF64(scratch, src1, src2);
  }
  BranchFalseF(scratch, &nan);

  if (kind == MaxMinKind::kMax) {
    if (std::is_same<float, F_TYPE>::value) {
      fmax_s(dst, src1, src2);
    } else {
      fmax_d(dst, src1, src2);
    }
  } else {
    if (std::is_same<float, F_TYPE>::value) {
      fmin_s(dst, src1, src2);
    } else {
      fmin_d(dst, src1, src2);
    }
  }
  jump(&done);

  bind(&nan);
  // if any operand is NaN, return NaN (fadd returns NaN if any operand is NaN)
  if (std::is_same<float, F_TYPE>::value) {
    fadd_s(dst, src1, src2);
  } else {
    fadd_d(dst, src1, src2);
  }

  bind(&done);
}

void MacroAssemblerRiscv64::Float32Max(FPURegister dst, FPURegister src1,
                                FPURegister src2) {
  comment(__FUNCTION__);
  FloatMinMaxHelper<float>(dst, src1, src2, MaxMinKind::kMax);
}

void MacroAssemblerRiscv64::Float32Min(FPURegister dst, FPURegister src1,
                                FPURegister src2) {
  comment(__FUNCTION__);
  FloatMinMaxHelper<float>(dst, src1, src2, MaxMinKind::kMin);
}

void MacroAssemblerRiscv64::Float64Max(FPURegister dst, FPURegister src1,
                                FPURegister src2) {
  comment(__FUNCTION__);
  FloatMinMaxHelper<double>(dst, src1, src2, MaxMinKind::kMax);
}

void MacroAssemblerRiscv64::Float64Min(FPURegister dst, FPURegister src1,
                                FPURegister src2) {
  comment(__FUNCTION__);
  FloatMinMaxHelper<double>(dst, src1, src2, MaxMinKind::kMin);
}

void MacroAssemblerRiscv64::BranchTrueShortF(Register rs, Label* target) {
  ma_branch(target, NotEqual, rs, Operand(zero_reg));
}

void MacroAssemblerRiscv64::BranchFalseShortF(Register rs, Label* target) {
  ma_branch(target, Equal, rs, Operand(zero_reg));
}

void MacroAssemblerRiscv64::BranchTrueF(Register rs, Label* target) {
  bool long_branch =
      target->bound() ? !is_near(target) : is_trampoline_emitted();
  if (long_branch) {
    Label skip;
    BranchFalseShortF(rs, &skip);
    BranchLong(target);
    bind(&skip);
  } else {
    BranchTrueShortF(rs, target);
  }
}

void MacroAssemblerRiscv64::BranchFalseF(Register rs, Label* target) {
  bool long_branch =
      target->bound() ? !is_near(target) : is_trampoline_emitted();
  if (long_branch) {
    Label skip;
    BranchTrueShortF(rs, &skip);
    BranchLong(target);
    bind(&skip);
  } else {
    BranchFalseShortF(rs, target);
  }
}


}  // namespace jit
}  // namespace js
