package Core.RTU

import chisel3._
import chisel3.util._
import Core.IntConfig._
import Core.ROBConfig._

class Cp0ToRtuBundle extends Bundle {
  val rtu_icg_en : Bool = Bool()
  val yy_clk_en : Bool = Bool()
}

class Entry extends Bundle {
  val reg : UInt = UInt(LogicRegsNumBits.W)
  val iid : UInt = UInt(InstructionIdNumBits.W)
  val preg : UInt = UInt(PhysicRegsNumBits.W)
}

class RetireControl(width: Int) extends Bundle {
  val async_flush : Bool = Bool()
  val retire_inst_preg_vld : Vec[Bool] = Vec(width, Bool())
  val iid : Vec[ValidIO[UInt]] = Vec(width, ValidIO(UInt(InstructionIdNumBits.W)))
}

class RobEntryInput extends Bundle {
  val dealloc_vld_for_gateclk : Bool            = Bool()
  val fromCp0                 : Cp0ToRtuBundle  = new Cp0ToRtuBundle
  val iduToRtu                : Vec[Entry]      = Vec(NumCreateEntry, new Entry)
  val ifu_xx_sync_reset       : Bool            = Bool()
  val pad_yy_icg_scan_en      : Bool            = Bool()
  val retireControl           : RetireControl   = new RetireControl(NumRetireEntry)
  val rtu_yy_xx_flush         : Bool            = Bool()
  val x_create_vld            : UInt            = UInt(NumCreateEntry.W)
  val x_dealloc_mask          : Bool            = Bool()
  val x_dealloc_vld           : Bool            = Bool()
  val x_release_vld           : Bool            = Bool()
  val x_reset_dst_reg         : UInt            = UInt(LogicRegsNumBits.W)
  val x_reset_mapped          : Bool            = Bool()
  val x_wb_vld                : Bool            = Bool()
}

class RobEntryOutput extends Bundle {
  val empty : Bool = Bool()
  val dreg_oh : UInt = UInt(LogicRegsNum.W)
  val preg_oh : UInt = UInt(PhysicRegsNum.W)
  val retired_released_wb : Bool = Bool()
}

class RobEntryIO extends Bundle {
  val in  : RobEntryInput = Input(new RobEntryInput)
  val out : RobEntryOutput = Output(new RobEntryOutput)
}

class RobEntry extends Module {
  val io : RobEntryIO = IO(new RobEntryIO)

  object PregState {
    def size = 5
    val dealloc :: wf_alloc :: alloc :: retire :: release :: Nil = Enum(size)
  }

  object WbState {
    def size = 2
    val idle :: wb :: Nil = Enum(size)
  }

  // Todo: Instance of Gated Cell

  private val lifecycle_cur_state = RegInit(PregState.dealloc)
  private val lifecycle_next_state = RegInit(PregState.dealloc)

  private val wb_cur_state = RegInit(WbState.idle)
  private val wb_next_state = RegInit(WbState.idle)

  private val reset_lifecycle_state = Mux(io.in.x_reset_mapped, PregState.retire, PregState.dealloc)
  private val wb_cur_state_wb = Wire(Bool())
  private val wb_cur_state_wb_masked = Wire(Bool())
  private val create_vld = WireInit(false.B)
  private val retire_vld = Wire(Bool())

  when(io.in.ifu_xx_sync_reset) {
    lifecycle_cur_state := reset_lifecycle_state
  }.otherwise {
    lifecycle_cur_state := lifecycle_next_state
  }

  switch(lifecycle_cur_state) {
    is(PregState.dealloc) {
      when(io.in.x_dealloc_mask) {
        lifecycle_next_state := PregState.wf_alloc
      }.otherwise {
        lifecycle_next_state := PregState.dealloc
      }
    }
    is(PregState.wf_alloc) {
      when(io.in.rtu_yy_xx_flush) {
        lifecycle_next_state := PregState.dealloc
      }.elsewhen(create_vld) {
        lifecycle_next_state := PregState.alloc
      }.otherwise {
        lifecycle_next_state := PregState.wf_alloc
      }
    }
    is(PregState.alloc) {
      when(io.in.rtu_yy_xx_flush) {
        lifecycle_next_state := PregState.dealloc
      }.elsewhen(io.in.x_release_vld && wb_cur_state_wb_masked) {
        lifecycle_next_state := PregState.dealloc
      }.elsewhen(io.in.x_release_vld) {
        lifecycle_next_state := PregState.release
      }.elsewhen(retire_vld) {
        lifecycle_next_state := PregState.retire
      }.otherwise{
        lifecycle_next_state := PregState.alloc
      }
    }
    is(PregState.retire) {
      when(io.in.x_release_vld && wb_cur_state_wb_masked) {
        lifecycle_next_state := PregState.dealloc
      }.elsewhen(io.in.x_release_vld) {
        lifecycle_next_state := PregState.release
      }.otherwise{
        lifecycle_next_state := PregState.retire
      }
    }
    is(PregState.release) {
      when(wb_cur_state_wb_masked) {
        lifecycle_next_state := PregState.dealloc
      }.otherwise{
        lifecycle_next_state := PregState.release
      }
    }
  }

  //==========================================================
  //               Preg Write Back State Machine
  //==========================================================
  private val reset_wb_state = Mux(io.in.x_reset_mapped, WbState.wb, WbState.idle)
  //----------------------------------------------------------
  //                 FSM of Preg write back
  //----------------------------------------------------------
  // State Description:
  // idle       : preg is not written back
  // wb         : preg is written back
  when(io.in.retireControl.async_flush) {
    wb_cur_state := WbState.wb
  }.elsewhen(io.in.ifu_xx_sync_reset) {
    wb_cur_state := reset_wb_state
  }.elsewhen(create_vld) {
    wb_cur_state := WbState.idle
  }.otherwise{
    wb_cur_state := wb_next_state
  }

  switch(wb_cur_state) {
    is(WbState.idle) {
      when(io.in.x_wb_vld) {
        wb_next_state := WbState.wb
      }.otherwise {
        wb_next_state := WbState.idle
      }
    }
    is(WbState.wb) {
      when(lifecycle_cur_state === PregState.dealloc) {
        wb_next_state := WbState.idle
      }.otherwise{
        wb_next_state := WbState.wb
      }
    }
  }

  //----------------------------------------------------------
  //                    Control Siganls
  //----------------------------------------------------------
  wb_cur_state_wb         := wb_cur_state === WbState.wb
  wb_cur_state_wb_masked  := wb_cur_state === WbState.wb && !io.in.x_dealloc_mask

  //==========================================================
  //                    Preg information
  //==========================================================
  //----------------------------------------------------------
  //              Prepare allocate create data
  //----------------------------------------------------------
  create_vld := io.in.x_create_vld.asUInt.orR

  private val create_entry = WireInit(0.U.asTypeOf(new Entry))
  private val entry = RegInit(0.U.asTypeOf(new Entry))

  create_entry := MuxLookup(io.in.x_create_vld, 0.U.asTypeOf(new Entry), Seq(
    "b0001".U -> io.in.iduToRtu(0),
    "b0010".U -> io.in.iduToRtu(1),
    "b0100".U -> io.in.iduToRtu(2),
    "b1000".U -> io.in.iduToRtu(3),
  ))

  //----------------------------------------------------------
  //                  Information Register
  //----------------------------------------------------------
  when(io.in.ifu_xx_sync_reset) {
    entry := 0.U.asTypeOf(new Entry)
    entry.reg := io.in.x_reset_dst_reg
  }.elsewhen(create_vld) {
    entry := create_entry
  }.otherwise {
    entry := entry
  }
  //----------------------------------------------------------
  //                Retire IID Match Register
  //----------------------------------------------------------
  // Todo: verilog: timing optimization: compare retire inst iid before retire

  val retire_inst_iid_match = WireInit(VecInit(Seq.fill(NumRetireEntry)(false.B)))

  when(io.in.ifu_xx_sync_reset) {
    retire_inst_iid_match := 0.U(retire_inst_iid_match.getWidth.W).asBools
  }.elsewhen(lifecycle_cur_state === PregState.alloc){
    for (i <- 0 until NumRetireEntry) {
      retire_inst_iid_match(i) := io.in.retireControl.iid(i).valid &&
        io.in.retireControl.iid(i).bits === entry.iid
    }
  }

  //==========================================================
  //                       Retire signal
  //==========================================================
  retire_vld :=
    VecInit(retire_inst_iid_match.zip(io.in.retireControl.retire_inst_preg_vld).map{
      case(iid_match, wb_vld) => iid_match && wb_vld
    }).asUInt.orR

  //==========================================================
  //                      Release signal
  //==========================================================
  io.out.preg_oh := Mux(lifecycle_cur_state === PregState.alloc && retire_vld, UIntToOH(entry.preg), 0.U)

  //==========================================================
  //              Rename Table Recovery signal
  //==========================================================
  io.out.dreg_oh := Mux(lifecycle_cur_state === PregState.retire, UIntToOH(entry.reg), 0.U)

  //==========================================================
  //          Fast Retired Instruction Write Back
  //==========================================================
  io.out.retired_released_wb := Mux(
    lifecycle_cur_state === PregState.alloc && retire_vld ||
      lifecycle_cur_state=== PregState.retire ||
      lifecycle_cur_state=== PregState.release,
    wb_cur_state === WbState.wb,
    true.B
  )
  io.out.empty := lifecycle_cur_state === PregState.dealloc

}
