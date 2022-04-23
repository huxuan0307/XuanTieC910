package Core.IU.Bju

import Core.AddrConfig.PcWidth
import Core.IUConfig.{PcFifoAddr, PcFifoLen}
import Utils.{CircularQueuePtr, HasCircularQueuePtrHelper}
import chisel3._
import chisel3.util._

class PcFifoPtr extends CircularQueuePtr[PcFifoPtr](PcFifoLen){

}

object PcFifoPtr {
  def apply(f: Bool, v:UInt): PcFifoPtr = {
    val ptr = Wire(new PcFifoPtr)
    ptr.flag := f
    ptr.value := v
    ptr
  }
}

class IduWriteIO extends Bundle{
  val writePcfifo  = Vec(2,Flipped(ValidIO(new IfuPredStore)))
  val alloPid      = Vec(2, Output(UInt(PcFifoAddr.W)))
  val isFull       = Output(Bool())
}
class PidIO extends Bundle{
  val read   = Input(UInt(PcFifoAddr.W))
  val write  = Input(UInt(PcFifoAddr.W))
}
class BjuRwIO extends Bundle {
  val readPcfifo  = Output(new IfuPredStore)
  val pid         = new PidIO
  val writePcfifo = Flipped(ValidIO(new PredCheckRes))
  val specialPid  = Input(UInt(PcFifoAddr.W))
  val specialPc   = Output(UInt(PcWidth.W))
}
class RobReadIO extends Bundle{
  val ifu_pred = Output(new IfuPredStore)
  val bht_check = Output(new PredCheckRes)
  val dealloPid = Input(Bool())
}
class PcFifoIO extends Bundle{
  val iduWrite  = new IduWriteIO // valid to out - pcfifo is full ? bht can not in : bht can in
  val bjuRw     = new BjuRwIO
  val robRead   = new RobReadIO
}

class PcFifo extends Module with HasCircularQueuePtrHelper{
  // PcFifo storage 3 data: chk_idx, bht_pred, jmp_mispred
  val io = IO(new PcFifoIO())
  val headPtr = RegInit(PcFifoPtr(false.B, 0.U))
  val tailPtr = RegInit(PcFifoPtr(true.B,  0.U))
  val is_full = distanceBetween(tailPtr, headPtr) < 2.U
  io.iduWrite.isFull := is_full
  val ifu_forward_data = io.iduWrite.writePcfifo
  // define 2 table
  var fifo_tab_pred  = RegInit(VecInit(Seq.fill(PcFifoLen)( 0.U.asTypeOf(new IfuPredStore) ))) // ifu pred data tab
  var fifo_tab_check = RegInit(VecInit(Seq.fill(PcFifoLen)( 0.U.asTypeOf(new PredCheckRes) ))) // bju jmp&br check tab
  //----------------------------------------------------------
  //               idu write in store & allocate ptr
  //----------------------------------------------------------
  for(i <- 0 until 2){
    val offset = if(i == 0) 0.U else PopCount(ifu_forward_data(i).valid)
    val ptr = headPtr + offset
    val idx = ptr.value
    fifo_tab_pred(idx) := ifu_forward_data(i).bits
    io.iduWrite.alloPid(i) := ptr.value
  }
  //----------------------------------------------------------
  //                   bju read by pid
  //----------------------------------------------------------
  val bju_read_pid = io.bjuRw.pid.read
  val readData = fifo_tab_pred(bju_read_pid) // VecInit(bju_read_pid.map(addr => fifo_tab_pred(addr)))
  io.bjuRw.readPcfifo := readData
  //----------------------------------------------------------
  //                  bju write by pid
  //----------------------------------------------------------
  val bju_write_pid = io.bjuRw.pid.write
  when(io.bjuRw.writePcfifo.valid){
    fifo_tab_check(bju_write_pid) := io.bjuRw.writePcfifo.bits
  }
  //----------------------------------------------------------
  //              rob read & deallocate ptr
  //----------------------------------------------------------
  when(io.robRead.dealloPid){
    io.robRead.bht_check :=  fifo_tab_check(tailPtr.value)
    io.robRead.ifu_pred  :=  fifo_tab_pred(tailPtr.value)
    val tailPtrNext = tailPtr + 1.U
    tailPtr := tailPtrNext
  }.otherwise{
    io.robRead.bht_check :=  DontCare
    io.robRead.ifu_pred  :=  DontCare
  }
  //----------------------------------------------------------
  //         Speical read, to Special Unit, auipc inst
  //----------------------------------------------------------
  io.bjuRw.specialPc  :=  fifo_tab_pred(io.bjuRw.specialPid).pc
}
