package Core.IFU
import Core.{Config, CoreBundle}
import chisel3._
import chisel3.util._
class RAS910IO extends CoreBundle {

  //val cp0_ifu_ras_en          = Input(Bool())

  val rtu_ifu_flush           = Input(Bool())
  val rtu_ifu_mispred         = Input(Bool())
  val rtu_ifu_pcall           = Input(Bool())
  val rtu_ifu_preturn         = Input(Bool())
  val ibdp_ras_push_pc        = Input(UInt(VAddrBits.W))
  val ibctrl_ras_pcall_vld    = Input(Bool())
  val ibctrl_ras_preturn_vld  = Input(Bool())
  val rtu_retire0_pc        = Input(UInt(VAddrBits.W))
  val ras_en                = Input(Bool())
  val pcall                 = Input(Bool())   //from ip call mask
  val preturn               = Input(Bool())

  val ras_data_vld            = Output(Bool())
  val ras_target_pc           = Output(UInt(VAddrBits.W))
  val ras_ubtb_pc           = Output(UInt(VAddrBits.W))
  val ras_ubtb_push_pc      = Output(UInt(VAddrBits.W))
  val ras_ubtb_push         = Output(Bool())
}
class RAS910 extends Module with Config {
  val io = IO(new RAS910IO)
  val ras = Mem(ifu_ras, UInt(VAddrBits.W))
  //ras fifo
  val ras_push = Wire(Bool())
  val ras_pop  = Wire(Bool())
  val ras_return = Wire(Bool())
  val ras_push_pc = Wire(UInt(VAddrBits.W))
  val ras_pc_out = Wire(UInt(VAddrBits.W))
  ras_push := io.ibctrl_ras_pcall_vld
  ras_pop  := io.ibctrl_ras_preturn_vld


  //rtu fifo
  val rtu_ifu_pcall = Wire(Bool())
  val rtu_ifu_preturn = Wire(Bool())
  rtu_ifu_pcall := io.rtu_ifu_pcall
  rtu_ifu_preturn := io.rtu_ifu_preturn

  val rtu_ptr     = RegInit(0.U(5.W))
  val rtu_ptr_pre = Wire(0.U(5.W))
  val ras_ptr     = RegInit(0.U(5.W))
  val ras_ptr_pre = Wire(0.U(5.W))
  val status_ptr  = RegInit(0.U(5.W))
  val status_ptr_pre = Wire(0.U(5.W))


  //rtu ptr
  val rtu_ras_empty = (rtu_ptr === status_ptr)
  when(rtu_ifu_preturn && rtu_ifu_pcall) {
    rtu_ptr_pre := rtu_ptr
  }.elsewhen(rtu_ifu_pcall) {
    when(rtu_ptr(3,0) === (ifu_ras-1).U) {
      rtu_ptr_pre := Cat(!rtu_ptr(4),"b0000".U)  //rtu_ras stack overflow
    }.otherwise {
      rtu_ptr_pre := rtu_ptr + 1.U
    }
  }.elsewhen(rtu_ifu_preturn && !rtu_ras_empty) {
    when(rtu_ptr(3,0) === 0.U) {
      rtu_ptr_pre := Cat(!rtu_ptr(4),"b1011".U)
    }.otherwise {
      rtu_ptr_pre := rtu_ptr - 1.U
    }
  }.otherwise {
    rtu_ptr_pre := rtu_ptr
  }
  rtu_ptr := Mux(io.ras_en,rtu_ptr_pre,rtu_ptr)

  //ras ptr
  val rtu_need = io.rtu_ifu_mispred || io.rtu_ifu_flush

  when(rtu_need) {
    ras_ptr_pre := rtu_ptr_pre
  }.elsewhen(ras_push && ras_return) {
    ras_ptr_pre := rtu_ptr
  }.elsewhen(ras_push) {
    when(ras_ptr(3,0) === (ifu_ras-1).U) {
      ras_ptr_pre := Cat(!ras_ptr(4), "b0000".U)
    }.otherwise {
      ras_ptr_pre := ras_ptr + 1.U
    }
  }.elsewhen(ras_pop && !ras_empty) {
    when(ras_ptr(3,0) === "b0000".U) {
      ras_ptr_pre := Cat(!ras_ptr(4), "b1011".U)
    }.otherwise {
      ras_ptr_pre := ras_ptr - 1.U
    }
  }.otherwise {
    ras_ptr_pre := ras_ptr
  }
  ras_ptr := Mux(io.ras_en,ras_ptr_pre,ras_ptr)

  //status ptr  used to solve ras overflow
  status_ptr_pre := Mux(status_ptr(3,0)==="b1011".U,Cat(status_ptr(4),"b0000".U),status_ptr+1.U)
  when(io.ras_en && rtu_need) {
    when(rtu_ptr_pre(4) ^ ras_ptr(4)) {
      status_ptr := 0.U
    }.otherwise {
      status_ptr := Cat(status_ptr(4), rtu_ptr_pre(3,0))
    }
  }.elsewhen(io.ras_en && ras_full && ras_push && ras_pop) {
    status_ptr := status_ptr
  }.elsewhen(io.ras_en && ras_full && ras_push) {
    status_ptr := status_ptr_pre
  }.elsewhen(io.ras_en && ras_full && ras_pop) {
    when(status_ptr(3,0) === 0.U) {
      status_ptr := 0.U
    }.otherwise {
      status_ptr := status_ptr - 1.U
    }
  }.otherwise {
    status_ptr := status_ptr
  }

  val ras_empty = ras_ptr === status_ptr
  val ras_full  = ras_ptr === Cat(!status_ptr(4),status_ptr(3,0))

  val rtu_fifo_ptr  = RegInit(0.U(4.W))
  val rtu_fifo_ptr_pre = Wire(0.U(4.W))
  rtu_fifo_ptr_pre := Mux(io.rtu_ifu_pcall,rtu_ptr(3,0),rtu_fifo_ptr(3,0))
  when(io.ras_en && io.rtu_ifu_pcall) {
    rtu_fifo_ptr := rtu_fifo_ptr_pre
  }


  //rtu fifo
  val rtu_push_index = Wire(0.U(6.W))
  for(i <- 0 until 6) {
    when(rtu_ptr(3,0) === (0+i).U || rtu_ptr(3,0) === (6+i).U) {
      rtu_push_index := i.U
    }
  }

  // ras push
  val pushindex = ras_ptr(3,0)
  val rtu_index = Wire(0.U(4.W))
  for(i <- 0 until 12) {
    when(rtu_fifo_ptr_pre === (0+i).U
    ||rtu_fifo_ptr_pre === (0+i).U(3,0) ||  rtu_fifo_ptr_pre === (1+i).U(3,0) ||  rtu_fifo_ptr_pre === (2+i).U(3,0)
      ||rtu_fifo_ptr_pre === (3+i).U(3,0) ||  rtu_fifo_ptr_pre === (4+i).U(3,0) ||  rtu_fifo_ptr_pre === (5+i).U(3,0)) {
      rtu_index := i.U
    }
  }
  val rtu_copy_index = rtu_index
  val ras_filled = rtu_need && (rtu_copy_index===ras_ptr(3,0))

  val ras_index = Mux(rtu_need,rtu_copy_index,ras_ptr(3,0))

  ras_push_pc := Mux(rtu_need,io.rtu_retire0_pc,io.ibdp_ras_push_pc)

  ras_pc_out            := ras.read(ras_index)
  when(ras_push || rtu_need){
    ras.write(ras_index,ras_push_pc)
  }
  io.ras_data_vld       := !ras_empty || ras_push && ras_filled
  io.ras_target_pc      := Mux(io.pcall,io.ibdp_ras_push_pc,ras_pc_out)
  io.ras_ubtb_pc        := ras_pc_out
  io.ras_ubtb_push_pc   := io.ibdp_ras_push_pc
  io.ras_ubtb_push      := ras_push && io.ras_en
}
