/* -*- Mode: C++; tab-width: 8; indent-tabs-mode: nil; c-basic-offset: 2 -*-
 * vim: set ts=8 sts=2 et sw=2 tw=80:
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */

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

void MacroAssemblerRiscv64::ma_compareF32(Register rd,
                                          DoubleCondition cc,
                                          FloatRegister cmp1,
                                          FloatRegister cmp2) {
  switch (cc) {
    case DoubleEqual:
      feq_s(rd, cmp1, cmp2);
      break;
    case DoubleNotEqual:
      feq_s(rd, cmp1, cmp2);
      NegateBool(rd, rd);
      break;
    case DoubleLessThan:
      flt_s(rd, cmp1, cmp2);
      break;
    case DoubleGreaterThanOrEqual:
      fle_s(rd, cmp2, cmp1);
      break;
    case DoubleLessThanOrEqual:
      fle_s(rd, cmp1, cmp2);
      break;
    case DoubleGreaterThan:
      flt_s(rd, cmp2, cmp1);
      break;
    default:
      MOZ_CRASH("UNREACHABLE");
  }
}

void MacroAssemblerRiscv64::ma_compareF64(Register rd,
                                          DoubleCondition cc,
                                          FloatRegister cmp1,
                                          FloatRegister cmp2) {
  switch (cc) {
    case DoubleEqual:
      feq_d(rd, cmp1, cmp2);
      break;
    case DoubleNotEqual:
      feq_d(rd, cmp1, cmp2);
      NegateBool(rd, rd);
      break;
    case DoubleLessThan:
      flt_d(rd, cmp1, cmp2);
      break;
    case DoubleGreaterThanOrEqual:
      fle_d(rd, cmp2, cmp1);
      break;
    case DoubleLessThanOrEqual:
      fle_d(rd, cmp1, cmp2);
      break;
    case DoubleGreaterThan:
      flt_d(rd, cmp2, cmp1);
      break;
    default:
      MOZ_CRASH("UNREACHABLE");
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
    ma_branch(fail, Equal, dest, Operand(kNegativeZero));
  }
  UseScratchRegisterScope temps(this);
  Register scratch = temps.Acquire();
  Trunc_w_d(dest, src, scratch);
  ma_branch(fail, Equal, scratch, Operand(0));
}

void MacroAssemblerRiscv64Compat::convertDoubleToPtr(FloatRegister src,
                                                     Register dest,
                                                     Label* fail,
                                                     bool negativeZeroCheck) {
  if (negativeZeroCheck) {
    fclass_d(dest, src);
    ma_branch(fail, Equal, dest, Operand(kNegativeZero));
  }
  UseScratchRegisterScope temps(this);
  Register scratch = temps.Acquire();
  Trunc_l_d(dest, src, scratch);
  ma_branch(fail, Equal, scratch, Operand(0));
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
    ma_branch(fail, Equal, dest, Operand(kNegativeZero));
  }
  UseScratchRegisterScope temps(this);
  Register scratch = temps.Acquire();
  Trunc_w_s(dest, src, scratch);
  ma_branch(fail, Equal, scratch, Operand(0));
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
  }
  add(dest, base, tmp);
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

void MacroAssemblerRiscv64Compat::incrementInt32Value(Address const&) {
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
    ma_branch(&end, ShortJump);
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
  ma_branch(&end, ShortJump);

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
  ma_branch(&end, ShortJump);

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
  ebreak();
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
  MOZ_CRASH();
}
CodeOffset MacroAssembler::call(Label*) {
  MOZ_CRASH();
}
CodeOffset MacroAssembler::call(Register) {
  MOZ_CRASH();
}
CodeOffset MacroAssembler::call(wasm::SymbolicAddress) {
  MOZ_CRASH();
}
CodeOffset MacroAssembler::callWithPatch() {
  MOZ_CRASH();
}
CodeOffset MacroAssembler::farJumpWithPatch() {
  MOZ_CRASH();
}
CodeOffset MacroAssembler::moveNearAddressWithPatch(Register) {
  MOZ_CRASH();
}
CodeOffset MacroAssembler::nopPatchableToCall() {
  MOZ_CRASH();
}
CodeOffset MacroAssembler::wasmTrapInstruction() {
  MOZ_CRASH();
}
size_t MacroAssembler::PushRegsInMaskSizeInBytes(LiveRegisterSet) {
  MOZ_CRASH();
}
template <typename T>
void MacroAssembler::branchValueIsNurseryCellImpl(Condition,
                                                  const T&,
                                                  Register,
                                                  Label*) {
  MOZ_CRASH();
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

uint32_t MacroAssembler::pushFakeReturnAddress(Register) {
  MOZ_CRASH();
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
void MacroAssembler::branchPtrInNurseryChunk(Condition,
                                             Register,
                                             Register,
                                             Label*) {
  MOZ_CRASH();
}
void MacroAssembler::branchTestValue(Condition,
                                     const ValueOperand&,
                                     const Value&,
                                     Label*) {
  MOZ_CRASH();
}
void MacroAssembler::branchValueIsNurseryCell(Condition,
                                              const Address&,
                                              Register,
                                              Label*) {
  MOZ_CRASH();
}
void MacroAssembler::branchValueIsNurseryCell(Condition,
                                              ValueOperand,
                                              Register,
                                              Label*) {
  MOZ_CRASH();
}
void MacroAssembler::call(const Address&) {
  MOZ_CRASH();
}
void MacroAssembler::call(ImmPtr) {
  MOZ_CRASH();
}
void MacroAssembler::call(ImmWord) {
  MOZ_CRASH();
}
void MacroAssembler::call(JitCode*) {
  MOZ_CRASH();
}
void MacroAssembler::callWithABINoProfiler(const Address&, MoveOp::Type) {
  MOZ_CRASH();
}
void MacroAssembler::callWithABINoProfiler(Register, MoveOp::Type) {
  MOZ_CRASH();
}
void MacroAssembler::callWithABIPost(uint32_t, MoveOp::Type, bool) {
  MOZ_CRASH();
}
void MacroAssembler::callWithABIPre(uint32_t*, bool) {
  MOZ_CRASH();
}
void MacroAssembler::ceilDoubleToInt32(FloatRegister, Register, Label*) {
  MOZ_CRASH();
}
void MacroAssembler::ceilFloat32ToInt32(FloatRegister, Register, Label*) {
  MOZ_CRASH();
}
void MacroAssembler::comment(const char*) {
  MOZ_CRASH();
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
void MacroAssembler::convertInt64ToDouble(Register64, FloatRegister) {
  MOZ_CRASH();
}
void MacroAssembler::convertInt64ToFloat32(Register64, FloatRegister) {
  MOZ_CRASH();
}
void MacroAssembler::convertIntPtrToDouble(Register, FloatRegister) {
  MOZ_CRASH();
}
void MacroAssembler::convertUInt64ToDouble(Register64,
                                           FloatRegister,
                                           Register) {
  MOZ_CRASH();
}
void MacroAssembler::convertUInt64ToFloat32(Register64,
                                            FloatRegister,
                                            Register) {
  MOZ_CRASH();
}
void MacroAssembler::copySignDouble(FloatRegister,
                                    FloatRegister,
                                    FloatRegister) {
  MOZ_CRASH();
}
void MacroAssembler::enterFakeExitFrameForWasm(Register,
                                               Register,
                                               ExitFrameType) {
  MOZ_CRASH();
}
void MacroAssembler::flexibleDivMod32(Register,
                                      Register,
                                      Register,
                                      bool,
                                      const LiveRegisterSet&) {
  MOZ_CRASH();
}
void MacroAssembler::flexibleQuotient32(Register,
                                        Register,
                                        bool,
                                        const LiveRegisterSet&) {
  MOZ_CRASH();
}
void MacroAssembler::flexibleRemainder32(Register,
                                         Register,
                                         bool,
                                         const LiveRegisterSet&) {
  MOZ_CRASH();
}
void MacroAssembler::floorDoubleToInt32(FloatRegister, Register, Label*) {
  MOZ_CRASH();
}
void MacroAssembler::floorFloat32ToInt32(FloatRegister, Register, Label*) {
  MOZ_CRASH();
}
void MacroAssembler::flush() {
  MOZ_CRASH();
}
void MacroAssembler::loadStoreBuffer(Register, Register) {
  MOZ_CRASH();
}
void MacroAssembler::moveValue(const TypedOrValueRegister&,
                               const ValueOperand&) {
  MOZ_CRASH();
}
void MacroAssembler::moveValue(const Value&, const ValueOperand&) {
  MOZ_CRASH();
}
void MacroAssembler::moveValue(const ValueOperand&, const ValueOperand&) {
  MOZ_CRASH();
}
void MacroAssembler::nearbyIntDouble(RoundingMode,
                                     FloatRegister,
                                     FloatRegister) {
  MOZ_CRASH();
}
void MacroAssembler::nearbyIntFloat32(RoundingMode,
                                      FloatRegister,
                                      FloatRegister) {
  MOZ_CRASH();
}
void MacroAssembler::oolWasmTruncateCheckF32ToI32(FloatRegister,
                                                  Register,
                                                  TruncFlags,
                                                  wasm::BytecodeOffset,
                                                  Label*) {
  MOZ_CRASH();
}
void MacroAssembler::oolWasmTruncateCheckF32ToI64(FloatRegister,
                                                  Register64,
                                                  TruncFlags,
                                                  wasm::BytecodeOffset,
                                                  Label*) {
  MOZ_CRASH();
}
void MacroAssembler::oolWasmTruncateCheckF64ToI32(FloatRegister,
                                                  Register,
                                                  TruncFlags,
                                                  wasm::BytecodeOffset,
                                                  Label*) {
  MOZ_CRASH();
}
void MacroAssembler::oolWasmTruncateCheckF64ToI64(FloatRegister,
                                                  Register64,
                                                  TruncFlags,
                                                  wasm::BytecodeOffset,
                                                  Label*) {
  MOZ_CRASH();
}
void MacroAssembler::patchCallToNop(uint8_t*) {
  MOZ_CRASH();
}
void MacroAssembler::patchCall(uint32_t, uint32_t) {
  MOZ_CRASH();
}
void MacroAssembler::patchFarJump(CodeOffset, uint32_t) {
  MOZ_CRASH();
}
void MacroAssembler::patchNearAddressMove(CodeLocationLabel,
                                          CodeLocationLabel) {
  MOZ_CRASH();
}
void MacroAssembler::patchNopToCall(uint8_t*, uint8_t*) {
  MOZ_CRASH();
}
void MacroAssembler::Pop(const ValueOperand&) {
  MOZ_CRASH();
}
void MacroAssembler::Pop(FloatRegister) {
  MOZ_CRASH();
}
void MacroAssembler::Pop(Register) {
  MOZ_CRASH();
}
void MacroAssembler::PopRegsInMaskIgnore(LiveRegisterSet, LiveRegisterSet) {
  MOZ_CRASH();
}
void MacroAssembler::popReturnAddress() {
  MOZ_CRASH();
}
void MacroAssembler::PopStackPtr() {
  MOZ_CRASH();
}
void MacroAssembler::PushBoxed(FloatRegister) {
  MOZ_CRASH();
}
void MacroAssembler::Push(const Imm32 imm) {
  MOZ_CRASH();
}
void MacroAssembler::Push(const ImmGCPtr imm) {
  MOZ_CRASH();
}
void MacroAssembler::Push(const ImmPtr ptr) {
  MOZ_CRASH();
}
void MacroAssembler::Push(const ImmWord imm) {
  MOZ_CRASH();
}
void MacroAssembler::Push(FloatRegister) {
  MOZ_CRASH();
}
void MacroAssembler::Push(Register) {
  MOZ_CRASH();
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
void MacroAssembler::pushReturnAddress() {
  MOZ_CRASH();
}
void MacroAssembler::roundDoubleToInt32(FloatRegister,
                                        Register,
                                        FloatRegister,
                                        Label*) {
  MOZ_CRASH();
}
void MacroAssembler::roundFloat32ToInt32(FloatRegister,
                                         Register,
                                         FloatRegister,
                                         Label*) {
  MOZ_CRASH();
}
void MacroAssembler::setupUnalignedABICall(Register) {
  MOZ_CRASH();
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
void MacroAssembler::truncDoubleToInt32(FloatRegister, Register, Label*) {
  MOZ_CRASH();
}
void MacroAssembler::truncFloat32ToInt32(FloatRegister, Register, Label*) {
  MOZ_CRASH();
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
    if (!BranchShortCheck(0, L, cond, rs, rt)) {
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
    }
  } else {
    if (is_trampoline_emitted() && jumpKind == LongJump) {
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
      BranchShort(L, cond, rs, rt);
    }
  }
}

// Branches when done from within riscv code.
void MacroAssemblerRiscv64::ma_b(Register lhs,
                                 ImmPtr imm,
                                 Label* l,
                                 Condition c,
                                 JumpKind jumpKind) {
  asMasm().ma_b(lhs, ImmWord(uintptr_t(imm.value)), l, c, jumpKind);
}

void MacroAssemblerRiscv64::ma_b(Register,
                                 ImmWord,
                                 Label*,
                                 Assembler::Condition,
                                 JumpKind) {
  MOZ_CRASH();
}

void MacroAssemblerRiscv64::ma_b(Register,
                                 Imm32,
                                 Label*,
                                 Assembler::Condition,
                                 JumpKind) {
  MOZ_CRASH();
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

void MacroAssemblerRiscv64::ma_sub64(Register rd, Register rs, Operand rt) {
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

void MacroAssemblerRiscv64::ma_cmp_set(Register,
                                       Register,
                                       Imm32,
                                       Assembler::Condition) {
  MOZ_CRASH();
}
void MacroAssemblerRiscv64::ma_jump(ImmPtr) {
  MOZ_CRASH();
}
void MacroAssemblerRiscv64::ma_lid(FloatRegister, double) {
  MOZ_CRASH();
}
void MacroAssemblerRiscv64::ma_lis(FloatRegister, float) {
  MOZ_CRASH();
}
void MacroAssemblerRiscv64::ma_load(Register,
                                    BaseIndex const&,
                                    LoadStoreSize,
                                    LoadStoreExtension) {
  MOZ_CRASH();
}
void MacroAssemblerRiscv64::ma_pop(FloatRegister) {
  MOZ_CRASH();
}
void MacroAssemblerRiscv64::ma_push(FloatRegister) {
  MOZ_CRASH();
}
}  // namespace jit
}  // namespace js
