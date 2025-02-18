package frc.robot.subsystems.batteryTracking;

import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructSerializable;
import java.nio.ByteBuffer;
import java.time.LocalDateTime;
import java.time.ZoneOffset;
import lombok.AllArgsConstructor;
import lombok.Getter;

@Getter
@AllArgsConstructor
public class LogEntry implements StructSerializable {
  private LocalDateTime dateTime;
  private double usageAH;
  public static LogEntryStruct struct = new LogEntryStruct();

  public static class LogEntryStruct implements Struct<LogEntry> {

    /**
     * Gets the Class object for the stored value.
     *
     * @return Class
     */
    @Override
    public Class<LogEntry> getTypeClass() {
      return LogEntry.class;
    }

    /**
     * Gets the type name (e.g. for schemas of other structs). This should be globally unique among
     * structs.
     *
     * @return type name
     */
    @Override
    public String getTypeName() {
      return "BatteryTrackingLogEntry";
    }

    /**
     * Gets the serialized size (in bytes). This should always be a constant.
     *
     * @return serialized size
     */
    @Override
    public int getSize() {
      return kSizeDouble * 2;
    }

    /**
     * Gets the schema.
     *
     * @return schema
     */
    @Override
    public String getSchema() {
      return "double epochMin;double usageAH";
    }

    /**
     * Deserializes an object from a raw struct serialized ByteBuffer starting at the current
     * position. Will increment the ByteBuffer position by getStructSize() bytes. Will not otherwise
     * modify the ByteBuffer (e.g. byte order will not be changed).
     *
     * @param bb ByteBuffer
     * @return New object
     */
    @Override
    public LogEntry unpack(ByteBuffer bb) {
      return new LogEntry(
          LocalDateTime.ofEpochSecond((long) bb.getDouble(), 0, ZoneOffset.UTC), bb.getDouble());
    }

    /**
     * Puts object contents to a ByteBuffer starting at the current position. Will increment the
     * ByteBuffer position by getStructSize() bytes. Will not otherwise modify the ByteBuffer (e.g.
     * byte order will not be changed).
     *
     * @param bb ByteBuffer
     * @param value object to serialize
     */
    @Override
    public void pack(ByteBuffer bb, LogEntry value) {
      bb.putDouble(value.getDateTime().toEpochSecond(ZoneOffset.UTC));
      bb.putDouble(value.getUsageAH());
    }
  }
}
