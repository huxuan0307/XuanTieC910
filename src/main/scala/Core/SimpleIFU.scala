package Core

import Core.IFU.{BhtPredDataForward, IFUIO}
import Utils.RAMHelper
import chisel3._
import chisel3.util.ValidIO

class SimpleIFU extends Module {
  val io: IFUIO = IO(new IFUIO)

  private val ram = Seq.fill(2)(Module(new RAMHelper))
  private val pc_update = Wire(UInt(32.W))
  private val pc = RegInit(0.U)
  private val data = Wire(Vec(2, UInt(64.W)))

  for (i <- 0 until 2) {
    ram(i).io.clk := clock
    ram(i).io.en := ~reset.asBool
    ram(i).io.wen := false.B
    ram(i).io.wIdx := DontCare
    ram(i).io.wdata := DontCare
    ram(i).io.wmask := DontCare
  }

  ram(0).io.rIdx := pc >> 3.U
  ram(1).io.rIdx := (pc + 8.U) >> 3.U
  data(0) := ram(0).io.rdata
  data(1) := ram(1).io.rdata

  for (i <- 0 until 3) {
    io.instData(i).pc := (pc + (i * 4).U)(15, 1) // not support compress insts
  }

  private val instVec = Wire(Vec(3, UInt(32.W)))

  when(pc(2) === 0.U) {
    instVec(0) := data(0)(31,0)
    instVec(1) := data(0)(63,32)
    instVec(2) := data(1)(31,0)
  }.otherwise {
    instVec(0) := data(0)(63,32)
    instVec(1) := data(1)(31,0)
    instVec(2) := data(1)(63,32)
  }

  private val cnt: UInt = RegInit(0.U(3.W))
  private val stall: Bool = Wire(Bool())

  when (cnt === 4.U) {
    cnt := 0.U
  }.otherwise {
    cnt := cnt + 1.U
  }

  private val redirect: ValidIO[UInt] = RegInit(0.U.asTypeOf(ValidIO(UInt(32.W))))
  redirect := io.bru_redirect
  stall := io.idu_ifu_id_stall || cnt =/= 4.U
  when (stall) {
    pc_update := pc
  }.elsewhen(redirect.valid) {
    pc_update := redirect.bits
  }.otherwise {
    pc_update := pc + (4*3).U
  }

  when (!stall) {
    pc := pc_update
  }

  for (i <- 0 until 3) {
    io.instData(i).vl_pred      := false.B
    io.instData(i).vl           := 0.U
    io.instData(i).vsew         := 0.U
    io.instData(i).vlmul        := 0.U

    io.instData(i).no_spec      := false.B
    io.instData(i).bkptb_inst   := false.B
    io.instData(i).bkpta_inst   := false.B
    io.instData(i).split_short  := false.B
    io.instData(i).fence        := false.B
    io.instData(i).split_long   := false.B
    io.instData(i).high_hw_expt := false.B
    io.instData(i).expt_vec     := 0.U
    io.instData(i).expt_vld     := false.B
    io.instVld(i)               := !stall
    io.instData(i).opcode       := instVec(i)
  }

  private val ifuForwardVec = Wire(Vec(3, new BhtPredDataForward))

  private val brVec = Wire(Vec(3, Bool()))
  brVec.zip(instVec).foreach {
    case (br, inst) =>
      br := inst(6, 0) === "b1100011".U
  }
  private val jalVec = Wire(Vec(3, Bool()))
  jalVec.zip(instVec).foreach {
    case (jal, inst) =>
      jal := inst(6, 0) === "b1101111".U
  }
  private val jalrVec = Wire(Vec(3, Bool()))
  jalrVec.zip(instVec).foreach {
    case (jalr, inst) =>
      jalr := inst(6, 0) === "b1100111".U && inst(14, 12) === "b000".U
  }

  private val pcfifoEnVec = Wire(Vec(3, Bool()))
  pcfifoEnVec.zipWithIndex.foreach {
    case (pcfifoEn, i) => pcfifoEn := brVec(i) || jalVec(i) || jalrVec(i)
  }


  for (i <- 0 until 3) {
    ifuForwardVec(i).en := pcfifoEnVec(i)
    ifuForwardVec(i).jal := jalVec(i)
    ifuForwardVec(i).jalr := jalrVec(i)
    ifuForwardVec(i).dstVld := false.B
    ifuForwardVec(i).predStore := 0.U.asTypeOf(chiselTypeOf(ifuForwardVec(i).predStore))
    ifuForwardVec(i).curPc := pc + (4 * i).U
    ifuForwardVec(i).tarPc := 0.U
  }

  when(ifuForwardVec(0).en) {
    io.ifuForward(0) := ifuForwardVec(0)
  }.elsewhen(ifuForwardVec(1).en) {
    io.ifuForward(0) := ifuForwardVec(1)
  }.elsewhen(ifuForwardVec(2).en) {
    io.ifuForward(0) := ifuForwardVec(2)
  }.otherwise {
    io.ifuForward(0) := DontCare
    io.ifuForward(0).en := false.B
  }

  when(ifuForwardVec(0).en && ifuForwardVec(1).en) {
    io.ifuForward(1) := ifuForwardVec(1)
  }.elsewhen(ifuForwardVec(0).en && ifuForwardVec(2).en) {
    io.ifuForward(1) := ifuForwardVec(2)
  }.elsewhen(ifuForwardVec(1).en && ifuForwardVec(2).en) {
    io.ifuForward(1) := ifuForwardVec(2)
  }.otherwise {
    io.ifuForward(1) := DontCare
    io.ifuForward(1).en := false.B
  }

  io.toROB.curPcLoad := 0.U
  io.toROB.curPc := 0.U
  io.pc := pc
  io.tlb.vaddr.valid := false.B
  io.tlb.vaddr.bits := 0.U

}