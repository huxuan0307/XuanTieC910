package Core.IS

import Core.ExceptionConfig._
import Core.IntConfig._
import Core.VectorUnitConfig._
import chisel3._
import chisel3.util._

trait LsiqConfig {
  def NumSrcLs = 3
}

object LsiqConfig extends LsiqConfig

class LsiqEntryData extends Bundle with LsiqConfig {

}
