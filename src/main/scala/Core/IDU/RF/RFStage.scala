package Core.IDU.RF

import Core.ExceptionConfig.ExceptionVecWidth
import Core.IDU.DecodeTable.{AluDecodeTable, BjuDecodeTable, DefaultInst}
import Core.IDU.{FuncUnit, IduFromRtuFlushBundle, ct_idu_rf_pipe2_decd}
import Core.IDU.IS.AiqConfig.NumSrcArith
import Core.IDU.IS.BiqConfig.NumSrcBr
import Core.IDU.IS.LsiqConfig.{NumSrcLs, NumSrcLsX}
import Core.IDU.IS.SdiqConfig.NumSrcSd
import Core.IDU.IS.VfiqConfig.NumSrcVf
import Core.IDU.IS._
import Core.IDU.Opcode.Opcode
import Core.IDU.RF.PrfConfig.NumPregReadPort
import Core.IntConfig._
import Core.PipelineConfig.NumPipeline
import Core.VectorUnitConfig._
import Utils.Bits.{sext, zext}
import chisel3._
import chisel3.util._

trait RFStageConfig {
  // aiq + biq + lsiq + sdiq + viq
  def NumIssueAiq = 2
  def NumIssueBiq = 1
  def NumIssueLsiq = 2
  def NumIssueSdiq = 1
  def NumIssueVfiq = 2
  def NumIssue : Int = NumIssueAiq + NumIssueBiq + NumIssueLsiq +
    NumIssueSdiq + NumIssueVfiq
  def NumAiqEntry = 8
  def NumBiqEntry = 12
  def NumLsiqEntry = 12
  def NumSdiqEntry = 12
  def NumVfiqEntry = 8
  // Todo
  def StorePipeNumSet = Seq(5)
  def LsuPipeNumSet = Seq(3, 4, 5)
  def BjuPipeNumSet = Seq(2)
}

class IssueEnBundle extends Bundle {
  val gateClkIssueEn  : Bool = Bool()
  val issueEn         : Bool = Bool()
}

class RFStageFromCp0Bundle extends Bundle {
  val iduIcgEn  : Bool = Bool()
  val yyClkEn   : Bool = Bool()
  val lsuFenceIBroadDis   : Bool = Bool()
  val lsuFenceRwBroadDis  : Bool = Bool()
  val lsuTlbBroadDis      : Bool = Bool()
}

class RFStageFromHpcpBundle extends Bundle {
  val iduCntEn  : Bool = Bool()
}

class RFStageFromIuBundle extends Bundle {
  val stall = new Bundle {
    val divWb   : Bool = Bool()
    val mulEx1  : Bool = Bool()
  }
}

class RFStageFromVfpuBundle extends Bundle {
  val stall = new Bundle {
    val vdivWb : Bool = Bool()
  }
}

class RFStageFromRtuBundle extends Bundle {
  val flush = new IduFromRtuFlushBundle
}

class RFStageCtrlInput extends Bundle with RFStageConfig {
  val fromCp0   = new RFStageFromCp0Bundle
  val fromHpcp  = new RFStageFromHpcpBundle
  val fromIu    = new RFStageFromIuBundle
  val issueEnVec : Vec[IssueEnBundle] = Vec(NumIssue, new IssueEnBundle)
  val fromPad   = new PrfFromPadBundle
  val fromRtu   = new RFStageFromRtuBundle
  val fromVfpu  = new RFStageFromVfpuBundle
}

class RFStageToIqBundle extends Bundle with RFStageConfig {
  val launchFailValid : Bool = Bool()
  val popValid        : Bool = Bool()
  // Todo: add forward signal
  //  e.g. ctrl_aiq0_rf_pipe0_alu_reg_fwd_vld ctrl_viq1_rf_pipe7_vmla_vreg_fwd_vld
  val popDlbValid     : Bool = Bool()
  val stall           : Bool = Bool()
  /**
   *   Only for sdiq
   */
  val stAddrReadySet  : Bool = Bool()
  // Todo: Add launch valid dup
  //  e.g. ctrl_xx_rf_pipe0_preg_lch_vld_dup0
}

class RFStageToCp0Bundle extends Bundle {
  // ctrl
  // Todo: figure out
  val gateClkSel : Bool = Bool()
  val sel     : Bool = Bool()
  // data
  val opcode  : UInt = UInt(Opcode.width.W)
  val iid     : UInt = UInt(InstructionIdWidth.W)
  // replace opcode with inst
  val inst    : UInt = UInt(InstBits.W)
  val preg    : UInt = UInt(NumPhysicRegsBits.W)
  val src0    : UInt = UInt(XLEN.W)
}

class RFStageToHpcpBundle extends Bundle with RFStageConfig {
  val instValid : Bool = Bool()
  class PipeSignal extends Bundle {
    val instValid           : Bool = Bool()
    val launchFailValid     : Bool = Bool()
    val regLaunchFailValid  : Bool = Bool()
  }
  val pipeVec : Vec[PipeSignal] = Vec(NumIssue, new PipeSignal)
}

class RFStageToExuGateClkBundle extends Bundle {
  // gateClkIssue
  // issue
  // gateClkSel
  // sel
  // cbusGateClkSel
}

class RFStageToFuCtrlBundle extends Bundle {
  val sel : Bool = Bool()
  val gateClkSel : Bool = Bool()
}

class RFStageCtrlOutput extends Bundle with RFStageConfig {
  val toIq    : Vec[RFStageToIqBundle] = Vec(NumIssue, new RFStageToIqBundle)
  val toCp0   = new RFStageToCp0Bundle
  val toHpcp  = new RFStageToHpcpBundle
  val toExu   = new RFStageToExuGateClkBundle
  val toBju   = new RFStageToFuCtrlBundle
  val toDiv   = new RFStageToFuCtrlBundle
  val toMul   = new RFStageToFuCtrlBundle
  val toSpecial = new RFStageToFuCtrlBundle
  val toAlu0  = new RFStageToFuCtrlBundle // pipe0
  val toAlu1  = new RFStageToFuCtrlBundle // pipe1
  val toLu    = new RFStageToFuCtrlBundle // pipe3
  val toSt    = new RFStageToFuCtrlBundle // pipe4
  val toSd    = new RFStageToFuCtrlBundle // pipe5
  val toVfpu0 = new RFStageToFuCtrlBundle // pipe6
  val toVfpu1 = new RFStageToFuCtrlBundle // pipe7
}

class RFStageCtrlBundle extends Bundle {
  val in  : RFStageCtrlInput  = Flipped(Output(new RFStageCtrlInput))
  val out : RFStageCtrlOutput = Output(new RFStageCtrlOutput)
}

class RFStageFromHadBundle extends Bundle {
  val iduWbBrData   : UInt = UInt(XLEN.W)
  val iduWbBrValid  : Bool = Bool()
}

class RFStageFromPadBundle extends Bundle {
  val yyIcgScanEn : Bool = Bool()
}

class RFStageDataInput extends Bundle with RFStageConfig {
  class AiqIssueBundle extends Bundle with RFStageConfig {
    val issueEntryOH    : UInt = UInt(NumAiqEntry.W)
    val issueReadData   : AiqEntryData = Output(new AiqEntryData)
    val issueEn         : Bool = Bool()
    val issueGateClkEn  : Bool = Bool()
  }
  class BiqIssueBundle extends Bundle with RFStageConfig {
    val issueEntryOH    : UInt = UInt(NumBiqEntry.W)
    val issueReadData   : BiqEntryData = Output(new BiqEntryData)
    val issueEn         : Bool = Bool()
    val issueGateClkEn  : Bool = Bool()
  }
  class LsiqIssueBundle extends Bundle with RFStageConfig {
    val issueEntryOH    : UInt = UInt(NumLsiqEntry.W)
    val issueReadData   : LsiqEntryData = Output(new LsiqEntryData)
    val issueEn         : Bool = Bool()
    val issueGateClkEn  : Bool = Bool()
  }
  class SdiqIssueBundle extends Bundle with RFStageConfig {
    val issueEntryOH    : UInt = UInt(NumSdiqEntry.W)
    val issueReadData   : SdiqEntryData = Output(new SdiqEntryData)
    val issueEn         : Bool = Bool()
    val issueGateClkEn  : Bool = Bool()
  }
  class VfiqIssueBundle extends Bundle with RFStageConfig {
    val issueEntryOH    : UInt = UInt(NumVfiqEntry.W)
    val issueReadData   : VfiqEntryData = Output(new VfiqEntryData)
    val issueEn         : Bool = Bool()
    val issueGateClkEn  : Bool = Bool()
  }

  val aiq0 = new AiqIssueBundle
  val aiq1 = new AiqIssueBundle
  val biq = new BiqIssueBundle
  val lsiq0 = new LsiqIssueBundle
  val lsiq1 = new LsiqIssueBundle
  val sdiq = new SdiqIssueBundle
  val vfiq0 = new VfiqIssueBundle
  val vfiq1 = new VfiqIssueBundle
  val fromHad = new RFStageFromHadBundle
  val fromPad = new RFStageFromPadBundle
}

class RFStageToFuDataBundle extends Bundle with RFStageConfig {
  val aluShort : Bool = Bool()
  val dstPreg = ValidIO(UInt(NumPhysicRegsBits.W))
  val dstVPreg = ValidIO(UInt(NumVPregsBits.W))
  val exceptVec = ValidIO(UInt(ExceptionVecWidth.W))
  // has replaced func with opcode
  val opcode      : UInt = UInt(Opcode.width.W)
  val highHwExpt  : Bool = Bool()
  val iid         : UInt = UInt(InstructionIdWidth.W)
  // Todo: imm
  val imm         : UInt = UInt(6.W)
  // output  [31 :0]  idu_iu_rf_pipe0_opcode;
  val inst        : UInt = UInt(InstBits.W)
  // Todo: imm
  // fifo id, ifu -> IU 拿到pid -> ifu
  val pid         : UInt = UInt(5.W)
  val rsltSel     : UInt = UInt(21.W)
  val specialImm  : UInt = UInt(20.W)
  val src0        : UInt = UInt(XLEN.W)
  val src1        : UInt = UInt(XLEN.W)
  val src1NoImm   : UInt = UInt(XLEN.W)
  val src2        : UInt = UInt(XLEN.W)
  val vl          : UInt = UInt(VlmaxBits.W)
  val vlmul       : UInt = UInt(VlmulBits.W)
  val vsew        : UInt = UInt(VsewBits.W)
  val offset = UInt(21.W)
}

class RFStageDataOutput extends Bundle with RFStageConfig {
  class RFStageToIqBundle(numEntry: Int, numSrc: Int) extends Bundle {
    val launchEntryOH : UInt = UInt(numEntry.W)
    val readyClr      : Vec[Bool] = Vec(numSrc, Bool())
  }
  val toAiq0  = new RFStageToIqBundle(NumAiqEntry, NumSrcArith)
  val toAiq1  = new RFStageToIqBundle(NumAiqEntry, NumSrcArith)
  val toBiq   = new RFStageToIqBundle(NumBiqEntry, NumSrcBr)
  val toLsiq0 = new RFStageToIqBundle(NumLsiqEntry, NumSrcLs)
  val toLsiq1 = new RFStageToIqBundle(NumLsiqEntry, NumSrcLs)
  val toSdiq  = new RFStageToIqBundle(NumSdiqEntry, NumSrcSd)
  val toVfiq0 = new RFStageToIqBundle(NumVfiqEntry, NumSrcVf)
  val toVfiq1 = new RFStageToIqBundle(NumVfiqEntry, NumSrcVf)

  val toIu0 = new RFStageToFuDataBundle
  val toIu1 = new RFStageToFuDataBundle
  val toBju = new RFStageToFuDataBundle
  /**
   * aluDstPregs(i): alu(i) dst preg
   */
  val aluDstPregs   : Vec[UInt] = Vec(NumAlu, UInt(NumPhysicRegsBits.W))

  /**
   * vfpuDstVregs(i): vfpu(i) dst preg
   */
  val vfpuDstVregs  : Vec[UInt] = Vec(NumVfUnit, UInt(NumVPregsBits.W))
}

class RFStageDataBundle extends Bundle {
  val r   : Vec[PrfReadBundle]  = Flipped(Vec(NumPregReadPort, new PrfReadBundle))
  val in  : RFStageDataInput    = Flipped(Output(new RFStageDataInput))
  val out : RFStageDataOutput   = Output(new RFStageDataOutput)
}

class RFStageIO extends Bundle {
  val data = new RFStageDataBundle
  val ctrl = new RFStageCtrlBundle
}

class RFStage extends Module with RFStageConfig {
  val io : RFStageIO = IO(new RFStageIO)

  private val rtu = io.ctrl.in.fromRtu
  private val aiq0_data = io.data.in.aiq0
  private val aiq1_data = io.data.in.aiq1
  private val biq_data = io.data.in.biq
  private val lsiq0_data = io.data.in.lsiq0
  private val lsiq1_data = io.data.in.lsiq1
  private val sdiq_data = io.data.in.sdiq
  private val vfiq0_data = io.data.in.vfiq0
  private val vfiq1_data = io.data.in.vfiq1

  private val pipeIssueEn = WireInit(VecInit(
    io.data.in.aiq0.issueEn,
    io.data.in.aiq1.issueEn,
    io.data.in.biq.issueEn,
    io.data.in.lsiq0.issueEn,
    io.data.in.lsiq1.issueEn,
    io.data.in.sdiq.issueEn,
    io.data.in.vfiq0.issueEn,
    io.data.in.vfiq1.issueEn,
  ))

  // Todo: more readable
  /**
   * readPortMap((i, j)): read port number of ith pipe, jth src
   */
  private val readPortMap = Map(
    (0, 0) -> 0,
    (0, 1) -> 1,
    (1, 0) -> 2,
    (1, 1) -> 3,
    (2, 0) -> 4,
    (2, 1) -> 5,
    (3, 0) -> 6,
    (3, 1) -> 7,
    (4, 0) -> 8,
    (4, 1) -> 9,
    (5, 0) -> 10,
    (0, 2) -> 7, // the same as pipe3 src1
    (1, 2) -> 10,// the same as pipe5 src0
  )

  //==========================================================
  //                          Regs
  //==========================================================

  // performance monitor
  private val pipeInstValidVec = RegInit(VecInit(Seq.fill(NumIssue)(false.B)))
  private val pipeLaunchFailValidVec = RegInit(VecInit(Seq.fill(NumIssue)(false.B)))
  private val lsuLaunchFailValidVec = RegInit(VecInit(Seq.fill(NumIssueLsiq + NumIssueSdiq)(false.B)))

  private val pipeIqEntriesOH : Vec[UInt] = RegInit(VecInit(
    0.U(NumAiqEntry.W),
    0.U(NumAiqEntry.W),
    0.U(NumBiqEntry.W),
    0.U(NumLsiqEntry.W),
    0.U(NumLsiqEntry.W),
    0.U(NumSdiqEntry.W),
    0.U(NumVfiqEntry.W),
    0.U(NumVfiqEntry.W),
  ))

  // Todo: imm
  private val pipeDstPregs = RegInit(VecInit(Seq.fill(8)(0.U(InstructionIdWidth.W))))

  private val prfSrcPregs = RegInit(VecInit(Seq.fill(NumPregReadPort)(0.U(NumPhysicRegsBits.W))))
  private val prfRdataVec = Wire(Vec(NumPregReadPort, UInt(XLEN.W)))
  prfRdataVec := io.data.r.map(_.data)

  // Todo: check imm NumPregReadPort
  private val fwdSrcPregs = RegInit(VecInit(Seq.fill(NumPregReadPort)(0.U(NumPhysicRegsBits.W))))

  //==========================================================
  //                 RF Inst Valid registers
  //==========================================================
  //----------------------------------------------------------
  //                Pipe0 Instruction Valid
  //----------------------------------------------------------
  private val aiq0IssueLaunchPreg = Wire(Bool())
  private val aiq0IssueSpecial = Wire(Bool())
  private val aiq0IssueDiv = Wire(Bool())
  aiq0IssueLaunchPreg := false.B //////todo: tem for firrtl
  aiq0IssueSpecial := false.B //////todo: tem for firrtl
  aiq0IssueDiv := false.B //////todo: tem for firrtl

  //alu and special should set ready for pipe0 ex2 fwd
  //pipe0 ex1 fwd is set by alu reg fwd vld

  private val aiq0IssueAluRegValid = io.ctrl.in.issueEnVec(0).issueEn &&
    io.data.in.aiq0.issueReadData.launchPreg
  private val aiq0IssueSpecialValid = io.ctrl.in.issueEnVec(0).issueEn &&
    io.data.in.aiq0.issueReadData.special
  // Todo: div gateclk
  private val iuIsDivIssue = io.ctrl.in.issueEnVec(0).issueEn &&
    io.data.in.aiq0.issueReadData.div

  private val pipe0PregLaunchValid = RegInit(false.B)
  private val pipe0SpecialValid = RegInit(false.B)

  //----------------------------------------------------------
  //                Pipe1 Instruction Valid
  //----------------------------------------------------------

  private val pipe1PregLaunchValid = RegInit(false.B)

  private val aiq1IssueAluRegValid = io.ctrl.in.issueEnVec(1).issueEn &&
    io.data.in.aiq1.issueReadData.launchPreg

  //----------------------------------------------------------
  //                Pipe2 Instruction Valid
  //----------------------------------------------------------

  //----------------------------------------------------------
  //                Pipe3 Instruction Valid
  //----------------------------------------------------------

  //----------------------------------------------------------
  //                Pipe4 Instruction Valid
  //----------------------------------------------------------

  //----------------------------------------------------------
  //                Pipe5 Instruction Valid
  //----------------------------------------------------------

  //----------------------------------------------------------
  //                Pipe6 Instruction Valid
  //----------------------------------------------------------

  private val pipe6vmlaPregLaunchValid = RegInit(false.B)
  private val viq0IssueVmlaRfValid = io.ctrl.in.issueEnVec(6).issueEn &&
    io.data.in.vfiq0.issueReadData.dstVregValid &&
    io.data.in.vfiq0.issueReadData.vmla

  //----------------------------------------------------------
  //                Pipe7 Instruction Valid
  //----------------------------------------------------------

  private val pipe7vmlaPregLaunchValid = RegInit(false.B)
  private val viq1IssueVmlaRfValid = io.ctrl.in.issueEnVec(7).issueEn &&
    io.data.in.vfiq1.issueReadData.dstVregValid &&
    io.data.in.vfiq1.issueReadData.vmla
  pipeInstValidVec.zipWithIndex.foreach {
    case (instValid, i) =>
      // pipe 5 rf stage is flush by flush_be
      // st pipe
      if (StorePipeNumSet.contains(i)) {
        when(rtu.flush.be) {
          instValid := false.B
        }.otherwise {
          instValid := io.ctrl.in.issueEnVec(i).issueEn
        }
      }
      else {
        when(rtu.flush.fe || rtu.flush.is) {
          instValid := false.B
        }.otherwise {
          instValid := io.ctrl.in.issueEnVec(i).issueEn
        }
      }
  }

  when(rtu.flush.fe || rtu.flush.is) {
    pipe0PregLaunchValid  := false.B
    pipe0SpecialValid     := false.B
    pipe1PregLaunchValid  := false.B
    pipe6vmlaPregLaunchValid := false.B
    pipe7vmlaPregLaunchValid := false.B
  }.otherwise {
    pipe0PregLaunchValid  := aiq0IssueAluRegValid
    pipe0SpecialValid     := aiq0IssueSpecialValid
    pipe1PregLaunchValid  := aiq1IssueAluRegValid
    pipe6vmlaPregLaunchValid := viq0IssueVmlaRfValid
    pipe7vmlaPregLaunchValid := viq1IssueVmlaRfValid
  }

  //==========================================================
  //                RF Launch Ready registers
  //==========================================================
  //----------------------------------------------------------
  //                Pipe0 Launch Ready valid
  //----------------------------------------------------------

  // Todo: fwd
  //  val pipe0AluRegFwdValidVec = Vec()

  private val aiq0IssueAluFwdInst = io.ctrl.in.issueEnVec(0).issueEn &&
    io.data.in.aiq0.issueReadData.dstValid &&
    io.data.in.aiq0.issueReadData.aluShort

  //----------------------------------------------------------
  //                Pipe1 Launch Ready valid
  //----------------------------------------------------------

  // Todo: fwd

  private val aiq1IssueAluFwdInst = io.ctrl.in.issueEnVec(1).issueEn &&
    io.data.in.aiq1.issueReadData.dstValid &&
    io.data.in.aiq1.issueReadData.aluShort
  // Todo:
//  val aiq1IssueMlaFwdInst = io.ctrl.in.issueEnVec(1).issueEn &&
//    io.data.in.aiq1.issueReadData.mlaValid

  //----------------------------------------------------------
  //                Pipe6 Launch Ready valid
  //----------------------------------------------------------

  // Todo: fwd

  private val viq0IssueVmlaFwdInst = io.ctrl.in.issueEnVec(6).issueEn &&
    io.data.in.vfiq0.issueReadData.vmlaShort

  //----------------------------------------------------------
  //                Pipe7 Launch Ready valid
  //----------------------------------------------------------

  // Todo: fwd

  private val viq1IssueVmlaFwdInst = io.ctrl.in.issueEnVec(7).issueEn &&
    io.data.in.vfiq1.issueReadData.vmlaShort

  //==========================================================
  //                     RF Stall Signals
  //==========================================================
  //----------------------------------------------------------
  //                     VR/GPR move stall
  //----------------------------------------------------------
  //ex2 pipe0/1 mtvr will share ex1 pipe6/7
  //stall viq all inst issue at pipe0/1 rf

  private val pipe0MtvrValid = pipeInstValidVec(0) && io.data.in.aiq0.issueReadData.mtvr
  private val pipe1MtvrValid = pipeInstValidVec(1) && io.data.in.aiq1.issueReadData.mtvr

  private val pipe6MfvrValid = pipeInstValidVec(6) && io.data.in.vfiq0.issueReadData.mfvr
  private val pipe7MfvrValid = pipeInstValidVec(7) && io.data.in.vfiq1.issueReadData.mfvr

  //----------------------------------------------------------
  //                     div/vdiv stall
  //----------------------------------------------------------
  //when div need to write back, it will stall issue at wb pipe
  private val pipe0DivStall = io.ctrl.in.fromIu.stall.divWb
  private val pipe6VdivStall = io.ctrl.in.fromVfpu.stall.vdivWb

  //----------------------------------------------------------
  //                        mul stall
  //----------------------------------------------------------
  private val pipe1MulStall = io.ctrl.in.fromIu.stall.mulEx1

  //----------------------------------------------------------
  //                 IU special cmplt stall
  //----------------------------------------------------------
  //when IU special need cmplt at EX1, it will share pipe0
  //RF cmplt port. it will not be conflict with wb port share,
  //so do not need lch fail
  private val pipe0SpecialStall = pipe0SpecialValid

  //----------------------------------------------------------
  //              Output Stall Signals to IQ
  //----------------------------------------------------------
  // Note: aiq0 --> viq0, aiq1 --> viq1
  // aiq0
  io.ctrl.out.toIq(0).stall := pipe6MfvrValid || pipe0DivStall || pipe0SpecialStall
  io.ctrl.out.toIq(1).stall := pipe7MfvrValid
  io.ctrl.out.toIq(2).stall := DontCare
  io.ctrl.out.toIq(3).stall := DontCare
  io.ctrl.out.toIq(4).stall := DontCare
  io.ctrl.out.toIq(5).stall := DontCare
  // viq0
  io.ctrl.out.toIq(6).stall := pipe0MtvrValid || pipe6VdivStall
  io.ctrl.out.toIq(7).stall := pipe1MtvrValid

  //==========================================================
  //                   Launch fail Signals
  //==========================================================
  private val pipeLaunchFailVec = Wire(Vec(NumIssue, Bool()))
  // Todo: figure this CAUTION
  //CAUTION: avoid dead lock: when inst1 lch fail inst2, inst2
  //         may lch fail inst1 through src no ready

  //----------------------------------------------------------
  //                  div/vdiv wb lch fail
  //----------------------------------------------------------
  //pipe6 vdiv stall may be conflict with pipe0 mtvr stall
  //pipe0 div stall may be conflict with pipe6 mfvr stall
  //so launch fail mfvr/mtvr at this situation
  private val pipe0VdivMtvrLaunchFail = pipe0MtvrValid && pipe6VdivStall
  private val pipe6DivMfvrLaunchFail = pipe6MfvrValid && pipe0DivStall

  //----------------------------------------------------------
  //                mult mfvr share lch fail
  //----------------------------------------------------------
  //pipe1 ex2 mult will share pipe1 rf
  //pipe7 ex2 mfvr will share pipe1 ex1,
  //  (which is pipe7 ex1 share pipe1 rf)
  //pipe1 ex2 mult will be conflict with pipe7 ex1 mfvr
  //so launch fail pipe7 rf mfvr when pipe1 ex1 is mult
  private val pipe7mulMfvrLaunchFail = pipe7MfvrValid && pipe1MulStall

  //----------------------------------------------------------
  //               preg read port share lch fail
  //----------------------------------------------------------
  //prepare src vld signals
  private val pipe0src2Valid = pipeInstValidVec(0) && io.data.in.aiq0.issueReadData.srcValid(2)
  private val pipe1src2Valid = pipeInstValidVec(1) && io.data.in.aiq1.issueReadData.srcValid(2)
  private val pipe3src1Valid = pipeInstValidVec(3) && io.data.in.lsiq0.issueReadData.srcValid(1)
  private val pipe5src0Valid = pipeInstValidVec(5) && io.data.in.sdiq.issueReadData.src0Valid
  //preg read port share priority:
  //pipe0 src2 > pipe3 src1, pipe1 src1 > pipe5 src0
  //so launch fail pipe3/5 when pipe0/1 shares preg read port
  private val pipe3PregLaunchFail = pipe3src1Valid && pipe0src2Valid
  private val pipe5PregLaunchFail = pipe5src0Valid && pipe1src2Valid

  //----------------------------------------------------------
  //               vreg read port share lch fail
  //----------------------------------------------------------
  private val pipe3srcVmValid = pipeInstValidVec(3) && io.data.in.lsiq0.issueReadData.srcVmValid
  private val pipe4srcVmValid = pipeInstValidVec(4) && io.data.in.lsiq1.issueReadData.srcVmValid
  private val pipe6srcV2Valid = pipeInstValidVec(6) && io.data.in.vfiq0.issueReadData.srcValidVec(2)
  private val pipe7srcV2Valid = pipeInstValidVec(7) && io.data.in.vfiq1.issueReadData.srcValidVec(2)

  //vreg read port share priority:
  //pipe6 srcv2 > pipe3 srcvm, pipe7 srcv2 > pipe4 srcvm
  //so launch fail pipe3/4 when pipe6/7 vreg read port need to shared
  private val pipe3VregLaunchFail = pipe3srcVmValid && pipe6srcV2Valid
  private val pipe4VregLaunchFail = pipe4srcVmValid && pipe7srcV2Valid

  //----------------------------------------------------------
  //               unsplit vmlu share lch fail
  //----------------------------------------------------------
  // Todo: figure out
  //need to consider pipe7 lch fail, avoiding dead lock:
  //if pipe7 vmul unsplit lch fail pipe6 older inst x, older inst x
  //may lch fail pipe7 vmul unsplit through src no ready

  private val pipe7VmulUnsplitValid = pipeInstValidVec(7) &&
    !pipeLaunchFailVec(7) &&
    io.data.in.vfiq1.issueReadData.vmulUnsplit
  private val pipe6VmulValid = pipeInstValidVec(7) &&
    io.data.in.vfiq0.issueReadData.vmul
  //pipe7 vmul unsplit inst will share pipe6 vmul
  //so lch fail pipe6 vmul at this situation
  private val pipe6vmulUnsplitLaunchFail = pipe7VmulUnsplitValid && pipe6VmulValid

  //----------------------------------------------------------
  //                 Source Not Ready Signal
  //----------------------------------------------------------
  private val pipeSrcNotReadyVec = Wire(Vec(NumIssue, Bool()))
  pipeSrcNotReadyVec.foreach(_ := false.B)
  // Todo:
  //----------------------------------------------------------
  //                 Launch Fail Signals
  //----------------------------------------------------------
  //lch fail without src no rdy
  private val pipeOtherLaunchFailVec = WireInit(VecInit(Seq.fill(NumIssue)(false.B)))
  pipeOtherLaunchFailVec := DontCare // 1,2,
  pipeOtherLaunchFailVec(0) := pipe0VdivMtvrLaunchFail
  pipeOtherLaunchFailVec(3) := pipe3PregLaunchFail || pipe3VregLaunchFail
  pipeOtherLaunchFailVec(4) := pipe4VregLaunchFail
  pipeOtherLaunchFailVec(5) := pipe5PregLaunchFail
  pipeOtherLaunchFailVec(6) := pipe6DivMfvrLaunchFail || pipe6vmulUnsplitLaunchFail
  pipeOtherLaunchFailVec(7) := pipe7mulMfvrLaunchFail

  //should consider src no ready lch fail
  pipeLaunchFailVec.zipWithIndex.foreach {
    case (launchFail, i) =>
      launchFail := pipeSrcNotReadyVec(i) || pipeOtherLaunchFailVec(i)
  }

  //==========================================================
  //                    RF Control Signals
  //==========================================================
  //----------------------------------------------------------
  //                    RF pipedown valid
  //----------------------------------------------------------
  private val pipeDownValidVec = Wire(Vec(NumIssue, Bool()))

  pipeDownValidVec.zipWithIndex.foreach {
    case (pipeDownValid, i) =>
      pipeDownValid := pipeInstValidVec(i) && !pipeLaunchFailVec(i)
  }

  //----------------------------------------------------------
  //                 lch fail to clear iq frz
  //----------------------------------------------------------
  io.ctrl.out.toIq.zipWithIndex.foreach {
    case (iq, i) =>
      iq.launchFailValid := pipeInstValidVec(i) && pipeLaunchFailVec(i)
  }

  //----------------------------------------------------------
  //               no lch fail to pop iq entry
  //----------------------------------------------------------
  io.ctrl.out.toIq.zipWithIndex.foreach {
    case (iq, i) =>
      //pipe3/4/5 pop by LSU
      if (!LsuPipeNumSet.contains(i))
        iq.popValid := pipeDownValidVec(i)
      else
        iq.popValid := DontCare

      //pop singals for IR dlb, optimized for timing
      if (!LsuPipeNumSet.contains(i) || !BjuPipeNumSet.contains(i))
        iq.popDlbValid := pipeInstValidVec(i)
      else
        iq.popDlbValid := DontCare
  }

  //----------------------------------------------------------
  //                    lch fail for hpcp
  //----------------------------------------------------------
  //lch fail by src no rdy
  io.ctrl.out.toHpcp.pipeVec.zipWithIndex.foreach {
    case (pipe, i) =>
      pipe.launchFailValid := pipeInstValidVec(i) && pipeSrcNotReadyVec(i)
  }
  //lch fail by preg share
  io.ctrl.out.toHpcp.pipeVec.foreach(_.regLaunchFailValid := false.B)
  io.ctrl.out.toHpcp.pipeVec(3).regLaunchFailValid := pipeInstValidVec(3) &&
    (pipe3PregLaunchFail || pipe3VregLaunchFail)
  io.ctrl.out.toHpcp.pipeVec(4).regLaunchFailValid := pipeInstValidVec(4) &&
    pipe4VregLaunchFail
  io.ctrl.out.toHpcp.pipeVec(5).regLaunchFailValid := pipeInstValidVec(5) &&
    pipe5PregLaunchFail

  //----------------------------------------------------------
  //                staddr rdy set for SDIQ
  //----------------------------------------------------------
  //staddr replayed after DC stage should not set stdata ready, because
  //its stdata already poped after staddr signal valid at DC stage
  io.ctrl.out.toIq.foreach(_.stAddrReadySet := false.B)
  io.ctrl.out.toIq(5).stAddrReadySet := pipeInstValidVec(4) &&
    !pipeLaunchFailVec(4) && io.data.in.lsiq1.issueReadData.stAddr

  //==========================================================
  //                    RF stage Decoder
  //==========================================================

  private val aluDecodeTable = AluDecodeTable.table
  private val bjuDecodeTable = BjuDecodeTable.table
  // Todo: lsuDecodeTable

  private val aiq0Inst = Wire(UInt(32.W))
  private val aiq0FU :: aiq0Op :: aiq0RdVld :: aiq0Rs1Vld :: aiq0Rs2Vld :: Nil = ListLookup(aiq0Inst, DefaultInst.inst, aluDecodeTable)

  // Todo: Add imm sel in decode table
  private val iu0_imm_sel = Wire(Vec(5, Bool()))
  iu0_imm_sel(0) := aiq0Inst(6,0) === "b0110111".U || aiq0Inst(6,0) === "b0010111".U
  iu0_imm_sel(1) := aiq0Inst(1,0) === "b11".U && !iu0_imm_sel(0)
  iu0_imm_sel(2) := DontCare
  iu0_imm_sel(3) := DontCare
  iu0_imm_sel(4) := DontCare

  // Todo: imm_sel for RV64C

  private val iu0_imm : UInt = MuxCase(0.U, Seq(
    iu0_imm_sel(0) -> zext(XLEN, aiq0Inst(31, 12)),
    iu0_imm_sel(1) -> sext(XLEN, aiq0Inst(31, 20)),
  ))

  private val biqInst = Wire(UInt(32.W))
  private val biqFu :: biqOp :: biqRd :: biqRs1 :: biqRs2 :: Nil = ListLookup(biqInst, DefaultInst.inst, bjuDecodeTable)

  private val pipe2_decd = Module(new ct_idu_rf_pipe2_decd)
  pipe2_decd.io.pipe2_decd_opcode := biqInst

  private val aiq1Inst = Wire(UInt(32.W))
  private val aiq1FU :: aiq1Op :: aiq1RdVld :: aiq1Rs1Vld :: aiq1Rs2Vld :: Nil = ListLookup(aiq1Inst, DefaultInst.inst, aluDecodeTable)

  // Todo: Add imm sel in decode table
  private val iu1_imm_sel = Wire(Vec(5, Bool()))
  iu1_imm_sel(0) := aiq1Inst(6,0) === "b0110111".U || aiq1Inst(6,0) === "b0010111".U
  iu1_imm_sel(1) := aiq1Inst(1,0) === "b11".U && !iu1_imm_sel(0)
  iu1_imm_sel(2) := DontCare
  iu1_imm_sel(3) := DontCare
  iu1_imm_sel(4) := DontCare

  // Todo: imm_sel for RV64C

  private val iu1_imm : UInt = MuxCase(0.U, Seq(
    iu1_imm_sel(0) -> zext(XLEN, aiq1Inst(31, 12)),
    iu1_imm_sel(1) -> sext(XLEN, aiq1Inst(31, 20)),
  ))


  //==========================================================
  //                   Pipeline Registers
  //==========================================================
  // Gathered from ct_idu_rf_dp.v 8 sub-segment

  // launchEntryOH
  io.data.out.toAiq0.launchEntryOH  := RegEnable(aiq0_data.issueEntryOH, 0.U.asTypeOf(aiq0_data.issueEntryOH),  aiq0_data.issueEn)
  io.data.out.toAiq1.launchEntryOH  := RegEnable(aiq1_data.issueEntryOH, 0.U.asTypeOf(aiq1_data.issueEntryOH),  aiq1_data.issueEn)
  io.data.out.toBiq.launchEntryOH   := RegEnable(biq_data.issueEntryOH, 0.U.asTypeOf(biq_data.issueEntryOH),  biq_data.issueEn)
  io.data.out.toLsiq0.launchEntryOH := RegEnable(lsiq0_data.issueEntryOH, 0.U.asTypeOf(lsiq0_data.issueEntryOH), lsiq0_data.issueEn)
  io.data.out.toLsiq1.launchEntryOH := RegEnable(lsiq1_data.issueEntryOH, 0.U.asTypeOf(lsiq1_data.issueEntryOH), lsiq1_data.issueEn)
  io.data.out.toSdiq.launchEntryOH  := RegEnable(sdiq_data.issueEntryOH, 0.U.asTypeOf(sdiq_data.issueEntryOH),  sdiq_data.issueEn)
  io.data.out.toVfiq0.launchEntryOH := RegEnable(vfiq0_data.issueEntryOH, 0.U.asTypeOf(vfiq0_data.issueEntryOH), vfiq0_data.issueEn)
  io.data.out.toVfiq1.launchEntryOH := RegEnable(vfiq1_data.issueEntryOH, 0.U.asTypeOf(vfiq1_data.issueEntryOH), vfiq1_data.issueEn)

  // update if issue enable
  private val aiq0ReadData  = RegEnable(io.data.in.aiq0.issueReadData, 0.U.asTypeOf(io.data.in.aiq0.issueReadData), io.data.in.aiq0.issueEn)
  private val aiq1ReadData  = RegEnable(io.data.in.aiq1.issueReadData, 0.U.asTypeOf(io.data.in.aiq1.issueReadData), io.data.in.aiq1.issueEn)
  private val biqReadData   = RegEnable(io.data.in.biq.issueReadData, 0.U.asTypeOf(io.data.in.biq.issueReadData), io.data.in.biq.issueEn)
  private val lsiq0ReadData = RegEnable(io.data.in.lsiq0.issueReadData, 0.U.asTypeOf(io.data.in.lsiq0.issueReadData), io.data.in.lsiq0.issueEn)
  private val lsiq1ReadData = RegEnable(io.data.in.lsiq1.issueReadData, 0.U.asTypeOf(io.data.in.lsiq1.issueReadData), io.data.in.lsiq1.issueEn)
  private val sdiqReadData  = RegEnable(io.data.in.sdiq.issueReadData, 0.U.asTypeOf(io.data.in.sdiq.issueReadData), io.data.in.sdiq.issueEn)

  aiq0Inst := aiq0ReadData.inst
  aiq1Inst := aiq1ReadData.inst
  biqInst  := biqReadData.inst

  io.data.out.aluDstPregs(0) := RegEnable(aiq0_data.issueReadData.dstPreg, 0.U.asTypeOf(aiq0_data.issueReadData.dstPreg), aiq0_data.issueEn)
  io.data.out.aluDstPregs(1) := RegEnable(aiq1_data.issueReadData.dstPreg, 0.U.asTypeOf(aiq1_data.issueReadData.dstPreg), aiq1_data.issueEn)
  io.data.out.vfpuDstVregs(0) := RegEnable(vfiq0_data.issueReadData.dstVreg, 0.U.asTypeOf(vfiq0_data.issueReadData.dstVreg), vfiq0_data.issueEn)
  io.data.out.vfpuDstVregs(1) := RegEnable(vfiq1_data.issueReadData.dstVreg, 0.U.asTypeOf(vfiq1_data.issueReadData.dstVreg), vfiq1_data.issueEn)

  private val issueEntriesOH = WireInit(VecInit(
    io.data.in.aiq0.issueEntryOH,
    io.data.in.aiq1.issueEntryOH,
    io.data.in.biq.issueEntryOH,
    io.data.in.lsiq0.issueEntryOH,
    io.data.in.lsiq1.issueEntryOH,
    io.data.in.sdiq.issueEntryOH,
    io.data.in.vfiq0.issueEntryOH,
    io.data.in.vfiq1.issueEntryOH,
  ))

  private val pipeDstPregsUpdate = WireInit(VecInit(
    io.data.in.aiq0.issueReadData.dstPreg,
    io.data.in.aiq1.issueReadData.dstPreg,
    0.U,
    io.data.in.lsiq0.issueReadData.dstPreg,
    io.data.in.lsiq1.issueReadData.dstPreg,
    0.U,
    io.data.in.vfiq0.issueReadData.dstPreg,
    io.data.in.vfiq1.issueReadData.dstPreg
  ))

  for (i <- 0 until 8) {
    when (pipeIssueEn(i)) {
      pipeIqEntriesOH(i) := issueEntriesOH(i)
      pipeDstPregs(i) := pipeDstPregsUpdate(i)
    }
  }

  //==========================================================
  //                    RF Read Regfile
  //==========================================================



  //----------------------------------------------------------
  //                  Read Regfile for IU
  //----------------------------------------------------------

//  io.data.r(0).preg := aiq0ReadData.srcVec(0).preg
//  io.data.r(1).preg := aiq0ReadData.srcVec(1).preg
//  // Todo: fix r(2), should be pipe3 src1 data
//  io.data.r(2).preg := aiq0ReadData.srcVec(2).preg
//  io.data.r.zipWithIndex.foreach {
//    case (rbundle, i) =>
//      if (i > 2)
//        rbundle.preg := DontCare
//  }

  //==========================================================
  //                RF Execution Unit Selection
  //==========================================================
  // Todo:

  //----------------------------------------------------------
  //               Pipe0 Exectuion Unit Selection
  //----------------------------------------------------------
  io.ctrl.out.toCp0.sel     := aiq0FU === FuncUnit.CP0      && pipeDownValidVec(0)
  io.ctrl.out.toAlu0.sel    := aiq0FU === FuncUnit.ALU      && pipeDownValidVec(0)
  io.ctrl.out.toSpecial.sel := aiq0FU === FuncUnit.SPECIAL  && pipeDownValidVec(0)
  io.ctrl.out.toDiv.sel     := aiq0FU === FuncUnit.DU       && pipeDownValidVec(0)

  io.ctrl.out.toCp0.gateClkSel      := aiq0FU === FuncUnit.CP0     && pipeInstValidVec(0)
  io.ctrl.out.toAlu0.gateClkSel     := aiq0FU === FuncUnit.ALU     && pipeInstValidVec(0)
  io.ctrl.out.toSpecial.gateClkSel  := aiq0FU === FuncUnit.SPECIAL && pipeInstValidVec(0)
  io.ctrl.out.toDiv.gateClkSel      := aiq0FU === FuncUnit.DU      && pipeInstValidVec(0)
  // Todo: cbus_gateclk_sel
  //  ct_idu_rf_ctrl.v:1431

  //----------------------------------------------------------
  //               Pipe1 Exectuion Unit Selection
  //----------------------------------------------------------
  io.ctrl.out.toAlu1.sel  := aiq1FU === FuncUnit.ALU && pipeDownValidVec(1)
  io.ctrl.out.toMul.sel   := aiq1FU === FuncUnit.MU  && pipeDownValidVec(1)

  io.ctrl.out.toAlu1.gateClkSel  := aiq1FU === FuncUnit.ALU && pipeInstValidVec(1)
  io.ctrl.out.toMul.gateClkSel   := aiq1FU === FuncUnit.MU  && pipeInstValidVec(1)
  // Todo: cbus_gateclk_sel

  //----------------------------------------------------------
  //               Pipe2 Exectuion Unit Selection
  //----------------------------------------------------------
  io.ctrl.out.toBju.sel         := pipeDownValidVec(2)
  io.ctrl.out.toBju.gateClkSel  := pipeInstValidVec(2)

  //----------------------------------------------------------
  //               Pipe3 Exectuion Unit Selection
  //----------------------------------------------------------
  io.ctrl.out.toLu.sel          := pipeDownValidVec(3)
  io.ctrl.out.toLu.gateClkSel   := pipeInstValidVec(3)

  //----------------------------------------------------------
  //               Pipe4 Exectuion Unit Selection
  //----------------------------------------------------------
  io.ctrl.out.toSt.sel          := pipeDownValidVec(4)
  io.ctrl.out.toSt.gateClkSel   := pipeInstValidVec(4)

  //----------------------------------------------------------
  //               Pipe5 Exectuion Unit Selection
  //----------------------------------------------------------
  io.ctrl.out.toSd.sel          := pipeDownValidVec(5)
  io.ctrl.out.toSd.gateClkSel   := pipeInstValidVec(5)

  //----------------------------------------------------------
  //               Pipe6 Exectuion Unit Selection
  //----------------------------------------------------------
  io.ctrl.out.toVfpu0.sel         := pipeDownValidVec(6)
  io.ctrl.out.toVfpu0.gateClkSel  := pipeInstValidVec(6)

  //----------------------------------------------------------
  //               Pipe6 Exectuion Unit Selection
  //----------------------------------------------------------
  io.ctrl.out.toVfpu1.sel         := pipeDownValidVec(7)
  io.ctrl.out.toVfpu1.gateClkSel  := pipeInstValidVec(7)

  //==========================================================
  //                   Performance Monitor
  //==========================================================
  private val pipeInstValid : Bool = pipeInstValidVec.reduce(_||_)
  private val pipeInstValidVecFF : Vec[Bool] = RegEnable(pipeInstValidVec, 0.U.asTypeOf(pipeInstValidVec), io.ctrl.in.fromHpcp.iduCntEn)
  io.ctrl.out.toHpcp.instValid := RegNext(pipeInstValid)
  io.ctrl.out.toHpcp.pipeVec.zipWithIndex.foreach {
    case (signal, i) => signal.instValid := pipeInstValidVecFF(i)
  }

  //----------------------------------------------------------
  //                    RF inst valid
  //----------------------------------------------------------
  // pipeInstValid and pipeInstValidVecFF have assigned

  //----------------------------------------------------------
  //              RF stage performance monitor
  //----------------------------------------------------------
  // Todo

  // Data Path
  val srcDataMap = Map(
    (0,0) -> Seq (
      aiq0ReadData.srcVec(0).wb -> prfRdataVec(readPortMap(0, 0)),  // has write back -> read reg
    ),
    (0,1) -> Seq (
      !aiq0ReadData.srcValid(1) -> iu0_imm,                         // !srcValid      -> use imm
      aiq0ReadData.srcVec(1).wb -> prfRdataVec(readPortMap(0, 1)),  // has write back -> use reg
    ),
    (0,2) -> Seq (
      aiq0ReadData.srcVec(2).wb -> prfRdataVec(readPortMap(0, 2)),  // has write back -> use reg
    ),
    (1,0) -> Seq (
      aiq1ReadData.srcVec(0).wb -> prfRdataVec(readPortMap(1, 0)),  // has write back -> read reg
    ),
    (1,1) -> Seq (
      !aiq1ReadData.srcValid(1) -> iu1_imm,                         // !srcValid      -> use imm
      aiq1ReadData.srcVec(1).wb -> prfRdataVec(readPortMap(1, 1)),  // has write back -> use reg
    ),
    (1,2) -> Seq (
      aiq0ReadData.srcVec(2).wb -> prfRdataVec(readPortMap(1, 2)),  // has write back -> use reg
    ),
    (2,0) -> Seq (
      biqReadData.srcVec(0).wb  -> prfRdataVec(readPortMap(2, 0)),  // has write back -> use reg
    ),
    (2,1) -> Seq (
      //Todo: fix this iu0_imm
      !biqReadData.srcValid(1)  -> iu0_imm,                         // !srcValid      -> use imm
      biqReadData.srcVec(1).wb  -> prfRdataVec(readPortMap(2, 1)),  // has write back -> use reg
    ),
    (3,0) -> Seq (
      lsiq0ReadData.srcVec(0).wb-> prfRdataVec(readPortMap(3, 0)),  // has write back -> use reg
    ),
    (3,1) -> Seq (
      lsiq0ReadData.srcVec(0).wb-> prfRdataVec(readPortMap(3, 1)),  // has write back -> use reg
    ),
    (4,0) -> Seq (
      lsiq1ReadData.srcVec(0).wb-> prfRdataVec(readPortMap(4, 0)),  // has write back -> use reg
    ),
    (4,1) -> Seq (
      lsiq1ReadData.srcVec(0).wb-> prfRdataVec(readPortMap(4, 1)),  // has write back -> use reg
    ),
    (5,0) -> Seq (
      sdiqReadData.src0.wb      -> prfRdataVec(readPortMap(5, 0)),  // has write back -> use reg
    )
  )

  //==========================================================
  //                    Pipe0 Data Path
  //==========================================================
  private val rfPipe0ClkEn = io.data.in.aiq0.issueGateClkEn
  // Todo: gated_clk_cell for pipe0

  //----------------------------------------------------------
  //                    Source Operand 0
  //----------------------------------------------------------
  // iu src0 only from reg
  io.data.out.toIu0.src0 := MuxCase(0.U, // Todo: replace with fwd data
    srcDataMap((0, 0))
  )

  //----------------------------------------------------------
  //                    Source Operand 1
  //----------------------------------------------------------
  // iu src1 from reg or imm
  io.data.out.toIu0.src1 := MuxCase(0.U, // Todo: replace with fwd data
    srcDataMap((0, 1))
  )
  // Todo: figure out src1NoImm
  io.data.out.toIu0.src1NoImm := MuxCase(0.U, // Todo: replace with fwd data
    Seq(
      aiq0ReadData.srcVec(1).wb -> prfRdataVec(readPortMap((0,1))),  // has write back -> use reg
    )
  )

  //----------------------------------------------------------
  //                    Source Operand 2
  //----------------------------------------------------------
  // iu src2 only from imm
  io.data.out.toIu0.src2 := MuxCase(0.U, // Todo: replace with fwd data
    srcDataMap((0, 2))
  )

  //----------------------------------------------------------
  //                 Source Not Ready Signal
  //----------------------------------------------------------
  //if source not ready, signal rf_ctrl launch fail and clear
  //issue queue ready

  private val iuSrcNoReadyVec = Wire(Vec(AiqConfig.NumSrcArith, Bool()))
  // src no ready when need read reg but data has not been write back
  iuSrcNoReadyVec.zipWithIndex.foreach {
    case (noReady, i) =>
      noReady := aiq0ReadData.srcValid(i) && !aiq0ReadData.srcVec(i).wb // Todo: && !fwd
  }

  pipeSrcNotReadyVec(0) := iuSrcNoReadyVec.reduce(_||_)
  io.data.out.toAiq0.readyClr.zipWithIndex.foreach {
    case (readyClr, i) =>
      readyClr := iuSrcNoReadyVec(i) && !pipeOtherLaunchFailVec(0)
  }

  //----------------------------------------------------------
  //                Output to Execution Units
  //----------------------------------------------------------
  io.data.out.toIu0.iid  := aiq0ReadData.iid
  io.data.out.toIu0.inst := aiq0ReadData.inst
  io.data.out.toIu0.dstPreg.valid := aiq0ReadData.dstValid
  io.data.out.toIu0.dstPreg.bits := aiq0ReadData.dstPreg
  io.data.out.toIu0.dstVPreg.valid := aiq0ReadData.dstVValid
  io.data.out.toIu0.dstVPreg.bits := aiq0ReadData.dstVreg
  io.data.out.toIu0.opcode := aiq0Op
  //  io.data.out.toIu.src0
  //  io.data.out.toIu.src1
  //  io.data.out.toIu.src2
  //  io.data.out.toIu.src1NoImm
  io.data.out.toIu0.imm := DontCare // Todo: figure out and fix
  io.data.out.toIu0.aluShort := aiq0ReadData.aluShort
  io.data.out.toIu0.rsltSel := DontCare // Todo: fix or remove
  io.data.out.toIu0.vlmul := aiq0ReadData.vlmul
  io.data.out.toIu0.vsew := aiq0ReadData.vsew
  io.data.out.toIu0.vl := aiq0ReadData.vl
  // output to special
  io.data.out.toIu0.exceptVec := aiq0ReadData.exceptVec
  io.data.out.toIu0.highHwExpt := aiq0ReadData.highHwExcept
  io.data.out.toIu0.specialImm := iu0_imm(19, 0)
  io.data.out.toIu0.pid := aiq0ReadData.pid
  io.data.out.toIu0.offset := DontCare
  // output to cp0
  io.ctrl.out.toCp0.iid := aiq0ReadData.iid
  io.ctrl.out.toCp0.inst := aiq0ReadData.inst
  io.ctrl.out.toCp0.preg := aiq0ReadData.dstPreg
  // Todo: Had
  io.ctrl.out.toCp0.src0 := io.data.r(0).data
  io.ctrl.out.toCp0.opcode := aiq0Op

  //==========================================================
  //                    Pipe1 Data Path
  //==========================================================
  private val rfPipe1ClkEn = io.data.in.aiq1.issueGateClkEn
  // Todo: gated_clk_cell for pipe1

  //----------------------------------------------------------
  //                    Source Operand 0
  //----------------------------------------------------------
  // iu src0 only from reg
  io.data.out.toIu1.src0 := MuxCase(0.U, // Todo: replace with fwd data
    srcDataMap((1, 0))
  )

  //----------------------------------------------------------
  //                    Source Operand 1
  //----------------------------------------------------------
  // iu src1 from reg or imm
  io.data.out.toIu1.src1 := MuxCase(0.U, // Todo: replace with fwd data
    srcDataMap((1, 1))
  )
  // Todo: figure out src1NoImm
  io.data.out.toIu1.src1NoImm := MuxCase(0.U, // Todo: replace with fwd data
    Seq(
      aiq0ReadData.srcVec(1).wb -> io.data.r(1).data,  // has write back -> use reg
    )
  )

  //----------------------------------------------------------
  //                    Source Operand 2
  //----------------------------------------------------------
  // iu src2 only from imm
  io.data.out.toIu1.src2 := MuxCase(0.U, // Todo: replace with fwd data
    srcDataMap((1, 2))
  )

  //----------------------------------------------------------
  //                 Source Not Ready Signal
  //----------------------------------------------------------
  //if source not ready, signal rf_ctrl launch fail and clear
  //issue queue ready

  private val iu1SrcNoReadyVec = Wire(Vec(AiqConfig.NumSrcArith, Bool()))
  // src no ready when need read reg but data has not been write back
  iu1SrcNoReadyVec.zipWithIndex.foreach {
    case (noReady, i) =>
      noReady := aiq1ReadData.srcValid(i) && !aiq1ReadData.srcVec(i).wb // Todo: && !fwd
  }

  pipeSrcNotReadyVec(1) := iu1SrcNoReadyVec.reduce(_||_)
  io.data.out.toAiq1.readyClr.zipWithIndex.foreach {
    case (readyClr, i) =>
      readyClr := iu1SrcNoReadyVec(i)
  }

  //----------------------------------------------------------
  //                Output to Execution Units
  //----------------------------------------------------------
  io.data.out.toIu1.iid  := aiq1ReadData.iid
  io.data.out.toIu1.inst := aiq1ReadData.inst
  io.data.out.toIu1.dstPreg.valid := aiq1ReadData.dstValid
  io.data.out.toIu1.dstPreg.bits := aiq1ReadData.dstPreg
  io.data.out.toIu1.dstVPreg.valid := aiq1ReadData.dstVValid
  io.data.out.toIu1.dstVPreg.bits := aiq1ReadData.dstVreg
  io.data.out.toIu1.opcode := aiq1Op
  //  io.data.out.toIu.src1
  //  io.data.out.toIu.src1
  //  io.data.out.toIu.src2
  //  io.data.out.toIu.src1NoImm
  io.data.out.toIu1.imm := DontCare // Todo: figure out and fix
  // Todo: mult
  //  io.data.out.toIu1.multFunc
  //  io.data.out.toIu1.mlaSrc2Preg
  //  io.data.out.toIu1.mlasrc2Vld
  io.data.out.toIu1.aluShort := aiq1ReadData.aluShort
  io.data.out.toIu1.rsltSel := DontCare // Todo: fix or remove
  io.data.out.toIu1.vlmul := aiq1ReadData.vlmul
  io.data.out.toIu1.vsew := aiq1ReadData.vsew
  io.data.out.toIu1.vl := aiq1ReadData.vl
  // no special
  io.data.out.toIu1.exceptVec   := DontCare
  io.data.out.toIu1.highHwExpt  := DontCare
  io.data.out.toIu1.specialImm  := DontCare
  io.data.out.toIu1.pid         := DontCare
  io.data.out.toIu1.offset := DontCare

  //==========================================================
  //                    Pipe2 Data Path
  //==========================================================
  private val rfPipe2ClkEn = biq_data.issueGateClkEn
  // Todo: gated_clk_cell for pipe2

  //----------------------------------------------------------
  //                    Source Operand 0
  //----------------------------------------------------------
  io.data.out.toBju.src0 := MuxCase(0.U,
    srcDataMap((2,0))
  )

  //----------------------------------------------------------
  //                    Source Operand 1
  //----------------------------------------------------------
  io.data.out.toBju.src1 := MuxCase(0.U,
    srcDataMap((2,1))
  )
  io.data.out.toBju.src2      := DontCare
  io.data.out.toBju.src1NoImm := DontCare

  //----------------------------------------------------------
  //                 Source Not Ready Signal
  //----------------------------------------------------------
  //if source not ready, signal rf_ctrl launch fail and clear
  //issue queue ready
  private val bjuSrcNoReadyVec = Wire(Vec(BiqConfig.NumSrcBr, Bool()))
  // src no ready when need read reg but data has not been write back
  bjuSrcNoReadyVec.zipWithIndex.foreach {
    case (noReady, i) =>
      noReady := biqReadData.srcValid(i) && !biqReadData.srcVec(i).wb // Todo: && !fwd
  }

  pipeSrcNotReadyVec(2) := bjuSrcNoReadyVec.reduce(_||_)
  io.data.out.toBiq.readyClr.zipWithIndex.foreach {
    case (readyClr, i) =>
      readyClr := bjuSrcNoReadyVec(i)
  }

  //----------------------------------------------------------
  //                Output to Execution Units
  //----------------------------------------------------------
  io.data.out.toBju.iid  := biqReadData.iid
  io.data.out.toBju.inst := biqReadData.inst
  io.data.out.toBju.opcode := biqOp
  //  io.data.out.toBju.src0
  //  io.data.out.toBju.src1
  // Todo: bju
  //  io.data.out.toBju.offset
  //  io.data.out.toBju.length
  //  io.data.out.toBju.rts
  //  io.data.out.toBju.pcall
  io.data.out.toBju.pid         := biqReadData.pid
  io.data.out.toBju.vlmul := aiq1ReadData.vlmul
  io.data.out.toBju.vsew := aiq1ReadData.vsew
  io.data.out.toBju.vl := aiq1ReadData.vl
  // no special
  io.data.out.toBju.exceptVec   := DontCare
  io.data.out.toBju.highHwExpt  := DontCare
  io.data.out.toBju.specialImm  := DontCare
  // no dst
  io.data.out.toBju.dstPreg     := DontCare
  io.data.out.toBju.dstVPreg    := DontCare
  io.data.out.toBju.imm         := DontCare
  // no alu
  io.data.out.toBju.aluShort    := DontCare
  io.data.out.toBju.rsltSel     := DontCare
  io.data.out.toBju.offset := pipe2_decd.io.pipe2_decd_offset

  //==========================================================
  //                Source Pipeline Registers
  //==========================================================
  // Gathered from ct_idu_rf_dp.v 8 sub-segment
  // pipe0
  private val rfPipe0PrfSrcPregUpdateVldVec = Wire(Vec(NumSrcArith, Bool()))
  private val rfPipe0FwdSrcPregUpdateVldVec = Wire(Vec(NumSrcArith, Bool()))
  rfPipe0PrfSrcPregUpdateVldVec(0) := pipeIssueEn(0)
  rfPipe0PrfSrcPregUpdateVldVec(1) := pipeIssueEn(0)
  rfPipe0PrfSrcPregUpdateVldVec(2) := pipeIssueEn(0) && aiq0_data.issueReadData.srcValid(2)
  rfPipe0FwdSrcPregUpdateVldVec(0) := pipeIssueEn(0)
  rfPipe0FwdSrcPregUpdateVldVec(1) := pipeIssueEn(0)
  rfPipe0FwdSrcPregUpdateVldVec(2) := pipeIssueEn(0) && aiq0_data.issueReadData.srcValid(2)

  when(rfPipe0PrfSrcPregUpdateVldVec(0)) {
    prfSrcPregs(readPortMap((0, 0))) := aiq0_data.issueReadData.srcVec(0).preg
  }
  when(rfPipe0PrfSrcPregUpdateVldVec(1)) {
    prfSrcPregs(readPortMap((0, 1))) := aiq0_data.issueReadData.srcVec(1).preg
  }
  when(rfPipe0FwdSrcPregUpdateVldVec(0)) {
    fwdSrcPregs(0) := aiq0_data.issueReadData.srcVec(0).preg
  }
  when(rfPipe0FwdSrcPregUpdateVldVec(1)) {
    fwdSrcPregs(1) := aiq0_data.issueReadData.srcVec(1).preg
  }

  // Todo: output

  // pipe1
  private val rfPipe1PrfSrcPregUpdateVldVec = Wire(Vec(NumSrcArith, Bool()))
  private val rfPipe1FwdSrcPregUpdateVldVec = Wire(Vec(NumSrcArith, Bool()))
  rfPipe1PrfSrcPregUpdateVldVec(0) := pipeIssueEn(1)
  rfPipe1PrfSrcPregUpdateVldVec(1) := pipeIssueEn(1)
  rfPipe1PrfSrcPregUpdateVldVec(2) := pipeIssueEn(1) && aiq1_data.issueReadData.srcValid(2)
  rfPipe1FwdSrcPregUpdateVldVec(0) := pipeIssueEn(1)
  rfPipe1FwdSrcPregUpdateVldVec(1) := pipeIssueEn(1)
  rfPipe1FwdSrcPregUpdateVldVec(2) := pipeIssueEn(1) && aiq1_data.issueReadData.srcValid(2)

  // Todo: imm
  for (src <- 0 until 2) {
    when(rfPipe1PrfSrcPregUpdateVldVec(src)) {
      prfSrcPregs(readPortMap((1, src))) := aiq1_data.issueReadData.srcVec(src).preg
    }
    when(rfPipe1FwdSrcPregUpdateVldVec(src)) {
      fwdSrcPregs(readPortMap((1, src))) := aiq1_data.issueReadData.srcVec(src).preg
    }
  }

  // Todo: output

  // pipe2
  private val rfPipe2PrfSrcPregUpdateVldVec = Wire(Vec(NumSrcBr, Bool()))
  private val rfPipe2FwdSrcPregUpdateVldVec = Wire(Vec(NumSrcBr, Bool()))
  rfPipe2PrfSrcPregUpdateVldVec.foreach(_ := pipeIssueEn(2))
  rfPipe2FwdSrcPregUpdateVldVec.foreach(_ := pipeIssueEn(2))
  for (src <- 0 until 2) {
    when(rfPipe2PrfSrcPregUpdateVldVec(src)) {
      prfSrcPregs(readPortMap((2, src))) := biq_data.issueReadData.srcVec(src).preg
    }
    when(rfPipe2FwdSrcPregUpdateVldVec(src)) {
      fwdSrcPregs(readPortMap((2, src))) := biq_data.issueReadData.srcVec(src).preg
    }
  }

  // pipe3
  private val rfPipe3PrfSrcPregUpdateVldVec = Wire(Vec(NumSrcLsX, Bool()))
  private val rfPipe3FwdSrcPregUpdateVldVec = Wire(Vec(NumSrcLsX, Bool()))
  rfPipe3PrfSrcPregUpdateVldVec.foreach(_ := pipeIssueEn(3))
  rfPipe3FwdSrcPregUpdateVldVec.foreach(_ := pipeIssueEn(3))
  // Todo: srcvm prf and fwd
  //  ct_idu_rf_dp.v: 2879

  // Todo: simplify
  when(rfPipe3PrfSrcPregUpdateVldVec(0)) {
    prfSrcPregs(readPortMap((3, 0))) := lsiq0_data.issueReadData.srcVec(0).preg
  }
  when(rfPipe0PrfSrcPregUpdateVldVec(2)) {
    prfSrcPregs(readPortMap((0, 2))) := aiq0_data.issueReadData.srcVec(2).preg
  }.elsewhen(rfPipe3PrfSrcPregUpdateVldVec(1)) {
    prfSrcPregs(readPortMap((3, 1))) := lsiq0_data.issueReadData.srcVec(1).preg
  }

  when(rfPipe3FwdSrcPregUpdateVldVec(0)) {
    fwdSrcPregs(readPortMap((3, 0))) := lsiq0_data.issueReadData.srcVec(0).preg
  }
  when(rfPipe0FwdSrcPregUpdateVldVec(2)) {
    fwdSrcPregs(readPortMap((0, 2))) := aiq0_data.issueReadData.srcVec(2).preg
  }.elsewhen(rfPipe3PrfSrcPregUpdateVldVec(1)) {
    fwdSrcPregs(readPortMap((3, 1))) := lsiq0_data.issueReadData.srcVec(1).preg
  }

  // Todo: output
  io.data.r.zipWithIndex.foreach {
    case (bundle, i) =>
      bundle.preg := prfSrcPregs(i)
  }

  io.data.out.toVfiq0 := DontCare
  io.data.out.toVfiq1 := DontCare
  io.data.out.toLsiq0 := DontCare
  io.data.out.toLsiq1 := DontCare
  io.data.out.toSdiq := DontCare
}
