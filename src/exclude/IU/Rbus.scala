package Core.IU

import Core.IU.Du.DuRegData
import Core.IUConfig
import chisel3._
import chisel3.util.{is, _}

class RbusIn extends Bundle {
  val aluIn     = Vec(2,new AluRegData)
  val muIn      = new MuRegData
  val duIn      = new DuRegData
  val specialIn = new SpecialRegData // althoug only need (data, dataVld, preg)
  val cp0In     = new Cp0ToRbus
}
class RbusOut extends Bundle with IUConfig{
  val wbPreg = UInt(7.W)
  val wbData = UInt(XLEN.W)
  val wbPregVld = Bool()
}

class RbusIO extends Bundle with IUConfig {
  val in    = Input(new RbusIn)
  val flush = Input(Bool())
  val out   = Output(Vec(IuPipeNum-1, new RbusOut))
}
class Rbus extends Module with IUConfig {
  val io = IO(new RbusIO)
  val pipe_rslt_vld = Seq.fill(IuPipeNum-1)(Wire(Bool()))
  val special_in = io.in.specialIn
  val cp0_in     = io.in.cp0In
  val alu_in     = io.in.aluIn
  val mu_in      = io.in.muIn
  val du_in      = io.in.duIn
  pipe_rslt_vld.head := alu_in(0).dataVld || du_in.pipe0DataVld || special_in.dataVld || special_in.dataVld
  pipe_rslt_vld(1)   := alu_in(1).dataVld || mu_in.dataVld

  //----------------------------------------------------------
  //                   Write Back Valid
  //----------------------------------------------------------
  val pipe_wb_vld = Seq.fill(IuPipeNum-1)(RegInit(false.B))
  for(i<- 0 until (IuPipeNum-1)){
    when(io.flush){
      pipe_wb_vld(i) := false.B
    }otherwise{
      pipe_wb_vld(i) := pipe_rslt_vld(i)
    }
  }
  //----------------------------------------------------------
  //                Write Back Data Selection
  //----------------------------------------------------------
  //----------------------------------------------------------
  //                         Pipe 0
  //----------------------------------------------------------
  val pipe0_select = Cat(alu_in(0).dataVld ,du_in.pipe0DataVld ,special_in.dataVld, cp0_in.rsltVld) // TODO add CP0
  val pipe0_rslt_preg = RegInit(0.U(7.W))
  switch(pipe0_select){
    is("b1000".U){
      pipe0_rslt_preg := alu_in(0).preg
  }
    is("b0100".U){
      pipe0_rslt_preg := du_in.preg
    }
    is("b0010".U){
      pipe0_rslt_preg := special_in.preg
    }
    is("b0001".U){
      pipe0_rslt_preg := cp0_in.rsltPreg
    }
  }
  val pipe0_rslt_data = RegInit(0.U(XLEN.W))
  switch(pipe0_select){
    is("b1000".U){
      pipe0_rslt_data := alu_in(0).data
    }
    is("b0100".U){
      pipe0_rslt_data := du_in.data
    }
    is("b0010".U){
      pipe0_rslt_data := special_in.data
    }
    is("b0001".U){
      pipe0_rslt_data  := cp0_in.rsltData
    }
  }
  val pipe0_wb_rslt_preg = RegEnable(pipe0_rslt_preg, pipe_rslt_vld.head)
  val pipe0_wb_rslt_data = RegEnable(pipe0_rslt_data, pipe_rslt_vld.head)
  io.out(0).wbPreg    := pipe0_wb_rslt_preg
  io.out(0).wbData    := pipe0_wb_rslt_data
  io.out(0).wbPregVld := pipe_wb_vld(0)
  //----------------------------------------------------------
  //                         Pipe 1
  //----------------------------------------------------------
  val pipe1_select = Cat(alu_in(1).dataVld ,mu_in.dataVld) // TODO add CP0
  val pipe1_rslt_preg = RegInit(0.U(7.W))
  switch(pipe1_select){
    is("b10".U){
      pipe1_rslt_preg := alu_in(1).preg
    }
    is("b01".U){
      pipe1_rslt_preg := mu_in.preg
    }
  }
  val pipe1_rslt_data = RegInit(0.U(XLEN.W))
  switch(pipe1_select){
    is("b10".U){
      pipe1_rslt_data := alu_in(1).data
    }
    is("b01".U){
      pipe1_rslt_data := mu_in.data
    }
  }
  val pipe1_wb_rslt_preg = RegEnable(pipe1_rslt_preg, pipe_rslt_vld(1))
  val pipe1_wb_rslt_data = RegEnable(pipe1_rslt_data, pipe_rslt_vld(1))
  io.out(1).wbPreg    := pipe1_wb_rslt_preg
  io.out(1).wbData    := pipe1_wb_rslt_data
  io.out(1).wbPregVld := pipe_wb_vld(1)

}
