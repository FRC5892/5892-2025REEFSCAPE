package frc.robot.util.BatteryTracking;

import java.io.StringWriter;
import java.nio.ByteBuffer;
import java.nio.CharBuffer;
import java.nio.charset.Charset;
import java.nio.charset.StandardCharsets;
import java.security.NoSuchAlgorithmException;
import java.time.LocalDateTime;
import java.time.format.DateTimeFormatter;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Iterator;
import java.util.List;
import java.util.function.DoubleSupplier;
import javax.smartcardio.*;

// TODO:
// - Docs
// - FIRST specific errors

/**
 * <strong>NFC Battery tracking & logging</strong><br>
 * <span>By FRC team 5892 Energy HEROs</span><br>
 * <span>Copyright 2024 under the MIT license</span><br>
 * <br>
 * <u>Intro</u><br>
 * This class will read and write a nfc tag with data including name, born year, usage data, and
 * more. This class is mostly not WPILIB specific. Once started, this class will automatically write
 * usage data to a tag at a customizable interval.
 *
 * <ul>
 *   <u>Hardware</u>
 *   <li>This class was designed and tested with MIFARE Classic® 1k tags. It should be customizable
 *       to any NFC-A type 2 tag, if not any NFC tag.
 *   <li>This class was designed and tested with the ACR122U reader, though the commands should be
 *       identical on any PC/SC reader.
 * </ul>
 *
 * <br>
 * <u>Installation</u><br>
 * TODO <br>
 * <u>Usage</u><br>
 * For normal autonomous operation, call {@link #initialRead()} then {@link
 * #updateAutonomously(DoubleSupplier)}. To manually update at the end of a match, for example, call
 * {@link #manualAsyncUpdate()}. To get information on the current battery, call {@link
 * #getInsertedBattery()}. <br>
 * <u>Customization</u><br>
 * This class can be heavily customized. Constants are stored at the top of each subclass. For tag
 * specifics, see the top of {@link NFCUtils}. For language & encoding, see the top of {@link
 * NDEFUtils}. For general options, see the top of {@link BatteryTracking}. Everything has
 * documentation.
 */
public class BatteryTracking {
  /** Error handler to use. */
  protected static ProgramSpecificErrorHandling ERROR_HANDLER = new JavaErrors();
  /** How often to write to the tag */
  private static final long WRITE_EVERY_SECS = 60 * 5;

  private static Battery insertedBattery;

  private static Thread asyncThread;

  /*
  Public API
   */

  /**
   * Start a thread to automatically update the battery
   *
   * @param usageSupplierAH supplier that is called every {@link #WRITE_EVERY_SECS}. This should get
   *     the <strong>TOTAL</strong> usage since power on, <strong>NOT</strong> the usage since last
   *     pull
   */
  public static void updateAutonomously(DoubleSupplier usageSupplierAH) {
    if (insertedBattery == null) {
      ERROR_HANDLER.consumeError("Call initialRead() before updateAutonomously(DoubleSupplier)!");
      return;
    }
    if (asyncThread == null) {
      asyncThread = new Thread(new BatteryRunner(usageSupplierAH, getInsertedBattery()));
      asyncThread.setName("Battery Tracking Thread");
      asyncThread.start();
    } else {
      ERROR_HANDLER.consumeError("Automatic writes thread attempted to start multiple times!");
    }
  }

  /**
   * Get a {@link Battery} object containing the inserted battery's information
   *
   * @return the inserted battery
   */
  public static Battery getInsertedBattery() {
    if (BatteryTracking.insertedBattery == null) {
      ERROR_HANDLER.consumeError("Tried to get inserted battery before initial read was called!");
    }
    return BatteryTracking.insertedBattery;
  }

  /**
   * Calls for a write on a separate thread.The thread must already be started. This could be useful
   * if the robot is about to be turned off.
   */
  public static void manualAsyncUpdate() {
    if (asyncThread == null) {
      ERROR_HANDLER.consumeError(
          "manualAsyncUpdate() called before updateAutonomously!", new NullPointerException());
    }
    // already checked for null so suppress
    //noinspection SynchronizeOnNonFinalField
    synchronized (asyncThread) {
      asyncThread.notifyAll();
    }
  }

  /**
   * Perform the Initial read of the card. This should be called before {@link
   * BatteryTracking#updateAutonomously(DoubleSupplier)}. <br>
   * This method assumes a tag is already in the correct spot
   */
  public static void initialRead() {
    if (BatteryTracking.insertedBattery == null) {
      try {
        ByteBuffer buffer = NFCUtils.dumpCard();
        String data = NDEFUtils.parseData(buffer);
        BatteryTracking.insertedBattery = Battery.fromTag(data);
      } catch (Exception e) {
        ERROR_HANDLER.consumeError("Failed to initially read battery", e);
      }
    } else {
      ERROR_HANDLER.consumeError("Initial read called twice!");
    }
  }

  /**
   * Update and write the card synchronously (blocking)
   *
   * @param usage Battery usage in AH of this session
   */
  public static void updateSync(double usage) {
    if (!insertedBattery.hasNewLog) {
      insertedBattery.newLog();
    }
    insertedBattery.getLog().get(0).update(usage);
    insertedBattery.sessionUsageAH = usage;
    try {
      BatteryTracking.write(insertedBattery.toTag());
    } catch (NoSuchAlgorithmException e) {
      ERROR_HANDLER.consumeError(
          "pcsclite daemon not found, is it installed?", e); // TODO: how to install pcsc
    } catch (CardException e) {
      ERROR_HANDLER.consumeError("Failed to write to card", e);
    }
  }

  /*
  Private API
   */

  /**
   * Write to the tag
   *
   * @param data serialized data to write
   * @throws CardException if sending commands fails
   * @throws NoSuchAlgorithmException If the pcsc daemon was not found.
   */
  private static void write(String data) throws NoSuchAlgorithmException, CardException {
    ByteBuffer buffer = NFCUtils.createBuffer();
    int length = NDEFUtils.ndefLength(data);
    NFCUtils.addMifareHeader((short) length, buffer);
    NDEFUtils.encode(data, buffer);
    NFCUtils.trimBuffer(buffer);
    NFCUtils.writeTag(buffer);
  }
  /**
   * Record for a new thread to write to the battery
   *
   * @param usageSupplierAH usage supplier, pulled every {@link #WRITE_EVERY_SECS}
   * @param battery battery instance to use
   */
  private record BatteryRunner(DoubleSupplier usageSupplierAH, Battery battery)
      implements Runnable {
    @Override
    public void run() {
      try {
        synchronized (Thread.currentThread()) {
          //noinspection InfiniteLoopStatement
          while (true) {
            // Wait
            Thread.currentThread().wait(WRITE_EVERY_SECS * 1000);
            // It was interrupted or finished waiting so write
            BatteryTracking.updateSync(usageSupplierAH.getAsDouble());
          }
        }
      } catch (InterruptedException e) {
        ERROR_HANDLER.consumeWarning("Automatic writing thread ended unexpectedly", e);
      }
    }
  }

  /** Utility class */
  private BatteryTracking() {}

  /** Represents a battery with its data */
  public static class Battery {

    private final int id;
    private final String name;
    private final int year;
    private final double testedCapacityAH;
    private double sessionUsageAH = 0;
    private double initialUsageAH;
    private List<LogEntry> log;
    private boolean hasNewLog = false;

    public Battery(
        int id,
        String name,
        int year,
        double testedCapacityAH,
        double initialUsageAH,
        List<LogEntry> log) {
      this.id = id;
      this.name = name;
      this.year = year;
      this.testedCapacityAH = testedCapacityAH;
      this.initialUsageAH = initialUsageAH;
      this.log = log;
      this.sessionUsageAH = 0;
    }
    /**
     * @return Battery's numeric ID
     */
    public int getId() {
      return id;
    }

    /**
     * @return Battery's friendly name
     */
    public String getName() {
      return name;
    }

    /**
     * @return Battery's start year
     */
    public int getYear() {
      return year;
    }

    /**
     * @return Battery's tested Capacity, in amp hours
     */
    public double getTestedCapacityAH() {
      return testedCapacityAH;
    }

    /**
     * @return Battery's usage this session, since it was last pulled
     */
    public double getSessionUsageAH() {
      return sessionUsageAH;
    }

    /**
     * @return Battery's usage before and during this session
     */
    public double getUsageAH() {
      return initialUsageAH + sessionUsageAH;
    }

    /**
     * @return Battery's usage before this session
     */
    public double getInitialUsageAH() {
      return initialUsageAH;
    }

    /**
     * @return Battery's log, represents each time it was used.
     */
    public List<LogEntry> getLog() {
      return log;
    }
    /**
     * Parse a tag Expects the format
     *
     * <pre>{@code
     * ID name year testedah
     * usedahu
     * YYMMDD HH:MM capacityAH
     * ...
     * }</pre>
     *
     * @param tagData Tag data to parse
     * @return Battery object equivalent to this data
     */
    private static Battery fromTag(String tagData) {
      Iterator<String> lines = tagData.lines().iterator();
      String line = lines.next();
      String[] header = line.split("\\s+");
      int batteryID = Integer.parseInt(header[0]);
      String batteryName = header[1];
      int year = Integer.parseInt(header[2]);
      // Strip the unit digits
      double testedCapacityAH = Double.parseDouble(header[3].substring(0, header[3].length() - 2));
      line = lines.next();
      double usedAH = Double.parseDouble(line.substring(0, line.length() - 3));
      List<LogEntry> log = new ArrayList<>();
      // Now parse the records
      while (lines.hasNext()) {
        line = lines.next();
        if (line.isBlank()) {
          break;
        }
        log.add(LogEntry.fromTag(line));
      }
      Collections.sort(log);
      return new Battery(batteryID, batteryName, year, testedCapacityAH, usedAH, log);
    }

    /**
     * Serializes a Battery to a String writable to a tag
     *
     * @return encoded String
     */
    private String toTag() {
      StringWriter writer = new StringWriter();
      writer.write(
          String.format(
              // All that header stuff
              "%03d %s %d %02.2fAH\n%02.2fAHu\n",
              id, name, year, testedCapacityAH, initialUsageAH + sessionUsageAH));
      for (LogEntry record : log) {
        writer.write(record.toTag() + "\n");
      }
      return writer.toString();
    }

    @Override
    public String toString() {
      return toTag();
    }

    /**
     * Create a new log, at the beginning. Deletes the oldest log entry if there is not enough space
     */
    private void newLog() {
      log.add(0, new LogEntry(LocalDateTime.now(), 0));
      if (!canAddMoreLogs()) {
        // remove the oldest log entry to make space
        log.remove(log.size() - 1);
      }
      hasNewLog = true;
    }

    private boolean canAddMoreLogs() {
      return (log.size() + 1) * LogEntry.SERIALIZED_SIZE
          < (NFCUtils.MAX_WRITABLE_BYTES - NDEFUtils.NDEF_MAX_HEADER_SIZE);
    }

    /** A Log entry, most likely every time the device was turned on. */
    public static class LogEntry implements Comparable<LogEntry> {
      private LocalDateTime dateTime;
      private double usageAH;
      // Wow this will fail at Y2.1K
      private static final String FORMATTER_PATTERN = "MMddyy HH:mm";
      private static final DateTimeFormatter FORMATTER =
          DateTimeFormatter.ofPattern(FORMATTER_PATTERN);
      private static final int SERIALIZED_SIZE = FORMATTER_PATTERN.length() + 7;

      /**
       * Create a new log entry with a given time and usage
       *
       * @param dateTime used time
       * @param usageAH usage during session
       */
      public LogEntry(LocalDateTime dateTime, double usageAH) {
        this.dateTime = dateTime;
        this.usageAH = usageAH;
      }

      /**
       * @return Date and Time of Log entry
       */
      public LocalDateTime getDateTime() {
        return dateTime;
      }

      /**
       * @return Usage during this session
       */
      public double getUsageAH() {
        return usageAH;
      }

      /**
       * Updates this log entry with a new usage number
       *
       * @param newUsageAH new usage
       */
      private void update(double newUsageAH) {
        dateTime = LocalDateTime.now();

        usageAH = newUsageAH;
      }
      /**
       * Deserializes a log entry from a line
       *
       * @param tagLine line to parse
       * @return log entry equivalent to line
       */
      private static LogEntry fromTag(String tagLine) {
        String[] splitLine = tagLine.split("\\s+");
        LocalDateTime dateTime = LocalDateTime.parse(splitLine[0] + " " + splitLine[1], FORMATTER);
        String usageField = splitLine[2];
        double usage = Double.parseDouble(usageField.substring(0, usageField.length() - 2));
        return new LogEntry(dateTime, usage);
      }

      /**
       * Serializes a log entry to a line
       *
       * @return Line equivalent to log
       */
      private String toTag() {
        return String.format("%s %02.2fAh", dateTime.format(FORMATTER), usageAH);
      }

      @Override
      public String toString() {
        return toTag();
      }

      @Override
      public int compareTo(LogEntry o) {
        return -this.dateTime.compareTo(o.dateTime);
      }
    }
  }

  /** NFC and Mifare utilities */
  private static class NFCUtils {
    /**
     * If this tag is MIFARE. Set to false if this is NTAG or something else generic This determines
     * decryption
     */
    private static final Boolean IS_MIFARE = true;
    /** Sectors on a card, 16 for MIFARE classic 1k */
    private static final int SECTORS = 16;
    /** Blocks per sector. For MIFARE classic 1k, this is 4. */
    private static final int BLOCKS_PER_SECTOR = 4;
    /**
     * Blocks to read in each sector. Mifare uses the last block for encryption stuff so only use
     * the first 3
     */
    @SuppressWarnings("ConstantConditions")
    private static final int BLOCKS_TO_USE_PER_SECTOR = IS_MIFARE ? 3 : 4;
    /** Bytes per block Mifare classic 1k has 16 */
    private static final int BYTES_PER_BLOCK = 16;
    /**
     * Milliseconds to wait for a card before erroring. It should already be close enough, so it
     * doesn't have to be large
     */
    private static final int WAIT_FOR_CARD_TIMEOUT_MS = 1_000;

    private static final int MAX_WRITABLE_BYTES =
        BLOCKS_TO_USE_PER_SECTOR * (SECTORS - 1) * BYTES_PER_BLOCK - 4;

    /** Utility class */
    private NFCUtils() {}

    /**
     * Connects to a card
     *
     * @return a card channel to communicate with the connected card
     * @throws CardException If a connection could not be established
     * @throws NoSuchAlgorithmException If the pcsc daemon was not found.
     */
    private static CardChannel connectToCard() throws CardException, NoSuchAlgorithmException {
      TerminalFactory terminalFactory = TerminalFactory.getInstance("PC/SC", null);
      CardTerminal terminal = terminalFactory.terminals().list().get(0);
      terminal.waitForCardPresent(WAIT_FOR_CARD_TIMEOUT_MS);

      Card card = terminal.connect("*");
      return card.getBasicChannel();
    }

    /**
     * Dump a connected card
     *
     * @return Bytebuffer of the card's contents
     * @throws CardException if sending command failed
     * @throws NoSuchAlgorithmException If the pcsc daemon was not found.
     */
    private static ByteBuffer dumpCard() throws CardException, NoSuchAlgorithmException {
      CardChannel channel = connectToCard();
      ByteBuffer bytes = createBuffer();
      long startTime = System.nanoTime();

      // don't read sector 0, it can't be written to or contain NDEF data, so I don't care
      for (int i = 1; i < SECTORS; i++) {
        readSector(bytes, i, channel);
      }
      if (!IS_MIFARE) return bytes; // If its not mifare we are done
      bytes.position(0);
      if (!(bytes.get() == (byte) 0x03))
        return bytes; // If it doesn't have these proprietary bytes were done
      byte length = bytes.get();
      if (length == (byte) 0xFF) {
        // multi byte length

        // more forward 2 extra bytes to chop
        bytes.position(bytes.position() + 2);
      }
      // chop
      bytes.slice();

      return bytes;
    }

    /**
     * Reads an individual sector of a card
     *
     * @param return_bytes ByteBuffer to append data to
     * @param sector sector id to read
     * @param channel card channel to communicate with the card
     * @throws CardException if the read command failed
     */
    private static void readSector(ByteBuffer return_bytes, int sector, CardChannel channel)
        throws CardException {
      if (IS_MIFARE) {
        // Will throw CardException if it fails, that's ok.
        authenticateSector(sector, channel);
      }
      // Reuse variables because java GC
      // Default to read position to -1 because we know that's not right
      byte[] commandBytes = new byte[] {(byte) 0xFF, (byte) 0xB0, 0x00, (byte) -1, BYTES_PER_BLOCK};
      CommandAPDU command;
      // Loop through every block and read it
      for (int i = sector * BLOCKS_PER_SECTOR;
          i < sector * BLOCKS_PER_SECTOR + BLOCKS_TO_USE_PER_SECTOR;
          i++) {
        try {
          // make new command
          commandBytes[3] = (byte) i;
          command = new CommandAPDU(commandBytes);

          ResponseAPDU response = channel.transmit(command);

          validateResponse(response); // Will return CardException, but it's in a try
          // Yay! add the good data
          byte[] data = response.getData();
          return_bytes.put(response.getData());
        } catch (Exception e) {
          // Pass it on with extra info
          throw new CardException(String.format("Could not read sector %d block %d", sector, i), e);
        }
      }
    }

    /**
     * Write data to a connected card
     *
     * @param bytes bytes to write
     * @throws CardException if sending commands fails
     * @throws NoSuchAlgorithmException If the pcsc daemon was not found
     */
    private static void writeTag(ByteBuffer bytes) throws CardException, NoSuchAlgorithmException {
      CardChannel channel = connectToCard();
      bytes.position(0);
      int sector = 1;
      while (bytes.hasRemaining()) {
        writeSector(bytes, sector, channel);
        sector++;
      }
    }

    /**
     * Write to a sector
     *
     * @param bytes bytes to write, at current position
     * @param sector sector number to write
     * @param channel card channel to communicate with the card
     * @throws CardException If writing failed
     */
    private static void writeSector(ByteBuffer bytes, int sector, CardChannel channel)
        throws CardException {
      try {
        if (IS_MIFARE) {
          authenticateSector(sector, channel);
        }
        ByteBuffer command = ByteBuffer.allocate(5 + BYTES_PER_BLOCK);
        command.put(new byte[] {(byte) 0xFF, (byte) 0xD6, 0x00, (byte) -1, (byte) BYTES_PER_BLOCK});
        ByteBuffer response =
            ByteBuffer.allocate(
                259); // We only use 2 bytes but for some reason CardChannel.transmit(Bytebuffer,
        // Bytebuffer) requires > 258
        for (int i = sector * BLOCKS_PER_SECTOR;
            i < sector * BLOCKS_PER_SECTOR + BLOCKS_TO_USE_PER_SECTOR && bytes.hasRemaining();
            i++) {
          try {
            // Write block

            // our block
            command.put(3, (byte) i);
            // write the next 16 block
            command.put(5, bytes, bytes.position(), BYTES_PER_BLOCK);
            // Since its absolute read, move the position ahead by BYTES_PER_BLOCK
            bytes.position(bytes.position() + BYTES_PER_BLOCK);

            command.position(0);

            channel.transmit(command, response);
            response.position(0);
            // Finally validate
            int sw1 = response.get(0) & 0xff;
            int sw2 = response.get(1) & 0xff;
            validateResponse(sw1, sw2);
          } catch (Exception e) {
            // Pass it on with extra info
            throw new CardException(String.format("Could not write block %d", i), e);
          }
        }
      } catch (Exception e) {
        throw new CardException(String.format("Could not write sector %d", sector), e);
      }
    }

    /**
     * Trims a bytebuffer to the next block
     *
     * @param bytes Trimmed ByteBuffer
     */
    private static void trimBuffer(ByteBuffer bytes) {
      int position = bytes.position();
      if (position % BYTES_PER_BLOCK != 0) {
        bytes.limit(((position / BYTES_PER_BLOCK) + 1) * BYTES_PER_BLOCK);
      } else {
        bytes.limit(position);
      }
    }

    /**
     * Creates a {@link ByteBuffer} long enough for all bytes on a card
     *
     * @return the new ByteBuffer
     */
    private static ByteBuffer createBuffer() {
      return ByteBuffer.allocate((SECTORS - 1) * BLOCKS_TO_USE_PER_SECTOR * BYTES_PER_BLOCK);
    }

    /**
     * Validates an APDU response Throws a CardException if it didn't succeed A success is 0x90 0x00
     *
     * @param response APDU response to check
     * @throws CardException if it wasn't successful
     */
    private static void validateResponse(ResponseAPDU response) throws CardException {
      validateResponse(response.getSW1(), response.getSW2());
    }

    /**
     * Validates an APDU response Throws a CardException if it didn't succeed A success is 0x90 0x00
     *
     * @param sw1 ADPU's sw1 byte (expected to be 0x90)
     * @param sw2 ADPU's sw2 byte (expected to be 0x00)
     * @throws CardException if it wasn't successful
     */
    private static void validateResponse(int sw1, int sw2) throws CardException {
      if (!responseSuccessful(sw1, sw2)) {
        throw new CardException(
            String.format("Expected success status codes 0x90 and 0x00, got %02X %02X", sw1, sw2));
      }
    }

    /**
     * Checks if an APDU response is successful
     *
     * @param response response to check
     * @return true if response was successful
     */
    private static boolean responseSuccessful(ResponseAPDU response) {
      return responseSuccessful(response.getSW1(), response.getSW2());
    }
    /**
     * Checks if an APDU response is successful
     *
     * @param sw1 ADPU's sw1 byte (expected to be 0x90)
     * @param sw2 ADPU's sw2 byte (expected to be 0x00)
     * @return true if response was successful
     */
    private static boolean responseSuccessful(int sw1, int sw2) {
      return sw1 == 0x90 && sw2 == 0x00;
    }

    /**
     * Authenticate to read/write to a sector
     *
     * @param sector Sector to authenticate
     * @param channel CardChannel to communicate with the card
     * @throws CardException If the command couldn't be sent or a sector didn't like our keys.
     */
    private static void authenticateSector(int sector, CardChannel channel) throws CardException {
      // Mifare command to authenticate. Don't ask me what it means
      // This trys the key (that's somewhere) as key B
      // My tags work with key B only so I try it first to increase speed.
      CommandAPDU command =
          new CommandAPDU(
              new byte[] {
                (byte) 0xFF,
                (byte) 0x86,
                0x00,
                0x00,
                0x05,
                0x01,
                0x00,
                (byte) (sector * BLOCKS_PER_SECTOR),
                0x61,
                0x00
              });
      ResponseAPDU response = channel.transmit(command);
      if (responseSuccessful(response)) return;
      // Store for logging
      int initialSW1 = response.getSW1();
      int initialSW2 = response.getSW2();
      // Reuse variables because java GC
      // Try as key A now
      command =
          new CommandAPDU(
              new byte[] {
                (byte) 0xFF,
                (byte) 0x86,
                0x00,
                0x00,
                0x05,
                0x01,
                0x00,
                (byte) (sector * BLOCKS_PER_SECTOR),
                0x60,
                0x00
              });
      response = channel.transmit(command);
      if (responseSuccessful(response)) return;
      throw new CardException(
          String.format(
              "Failed to decrypt. First got bytes: %02X %02X. Second got bytes: %02X %02X",
              initialSW1, initialSW2, response.getSW1(), response.getSW2()));
    }

    /**
     * Adds a Mifare NDEF header to a bytebuffer
     *
     * @param length length of NDEF data, excluding the terminator
     * @param buffer buffer to add header to, at current position
     */
    public static void addMifareHeader(short length, ByteBuffer buffer) {
      if (!IS_MIFARE) return;
      buffer.put((byte) 0x03);
      if (length <= 0xFE) {
        // 1 byte, short
        buffer.put((byte) length);
      } else {
        // 3 bytes, long
        buffer.put((byte) 0xFF);
        buffer.putShort(length);
      }
    }
  }

  /** Utility class for parsing NDEF data, specifically text records. */
  private static class NDEFUtils {
    /** ISO 639 code for encoded data. {@code en} would be appropriate */
    private static final String LANGUAGE_CODE = "en";
    /** Byte array for {@link #LANGUAGE_CODE}. This is what is actually used in code. */
    private static final byte[] LANGUAGE_CODE_BYTES = LANGUAGE_CODE.getBytes();
    /** Character set. I don't think this should be changed from UTF-8 */
    private static final Charset CHARSET = StandardCharsets.UTF_8;

    private static final int NDEF_MAX_HEADER_SIZE = 9 + LANGUAGE_CODE_BYTES.length;

    /**
     * Parses a NDEF text record
     *
     * @param data ByteBuffer of data to parse, starting at the currant position
     * @return the data of the record as a String
     * @throws NDEFException if parsing fails
     */
    private static String parseData(ByteBuffer data) throws NDEFException {
      long length = validateHeaderGetBodyLength(data);
      ByteBuffer payload = data.slice(); // get only the payload data, after the header
      payload.limit((int) length); // discard the rest of the card
      // actually decode
      ;
      CharBuffer chars = CHARSET.decode(payload);
      return chars.toString();
    }

    /**
     * Encodes a NDEF text record
     *
     * @param data String of data to encode to a text record
     * @param buffer ByteBuffer to put encoded data
     */
    private static void encode(String data, ByteBuffer buffer) {
      byte[] binData = data.getBytes(StandardCharsets.UTF_8);
      int length = binData.length;
      boolean isShort = length < 256;
      if (isShort) {
        buffer.put((byte) 0b11010001); // 1. Header: Message Start Message End, Well Known, Short
      } else {
        buffer.put((byte) 0b11000001); // 1. Header: Message Start Message End, Well Known, Long
      }
      buffer.put((byte) 1); // 2. Type length
      if (isShort) {
        buffer.put((byte) (length + LANGUAGE_CODE_BYTES.length + 1)); // 3. Length
      } else {
        buffer.putInt(length + LANGUAGE_CODE_BYTES.length + 1); // 3-6. payload length
      }
      buffer.put((byte) 'T'); // 4/7. payload type
      buffer.put((byte) LANGUAGE_CODE_BYTES.length); // 5/8. Text record: lang length
      buffer.put(LANGUAGE_CODE_BYTES); // Text record: lang
      buffer.put(binData); // data
      buffer.put((byte) 0xFE); // Terminator
    }

    /**
     * Gets the length of an NDEF record for given data, excluding terminator
     *
     * @param data data to check length of
     * @return length of NDEF record, excluding terminator
     */
    private static int ndefLength(String data) {
      // See encode(String, ByteBuffer) comments to see where the numbers came from
      int length = data.length();
      if (length < 256) {
        return 5 + LANGUAGE_CODE_BYTES.length + length;
      } else {
        return 8 + LANGUAGE_CODE_BYTES.length + length;
      }
    }

    /**
     * Validates a NDEF header and returns the payload length
     *
     * @param data bytes to read header from
     * @return length of NDEF payload
     * @throws NDEFException if parsing fails
     */
    private static long validateHeaderGetBodyLength(ByteBuffer data) throws NDEFException {
      byte flags_byte = data.get();
      if (flags_byte != (byte) 0b11000001 && flags_byte != (byte) 0b11010001) {
        throw new RuntimeException(
            String.format(
                "Expected 1st byte of NDEF to be 11000001 or 11010001 but found %s. Tag must contain exactly 1 Text LogEntry",
                Integer.toBinaryString(flags_byte)));
      }
      boolean isShort = (((byte) (flags_byte >>> 4)) & 1) == 1;
      byte type_length = data.get();
      if (type_length != 1) {
        throw new NDEFException("Tag must contain a Text LogEntry");
      }
      long length;
      if (isShort) {
        length =
            data.get() & 0xFF; // Java signed bytes :( https://stackoverflow.com/questions/4266756/
      } else {
        length = Integer.toUnsignedLong(data.getInt());
      }
      char payloadType = (char) data.get();
      if (payloadType != 'T') {
        throw new NDEFException("Tag must contain a Text LogEntry");
      }
      byte languageLength = data.get();
      if (languageLength != LANGUAGE_CODE.length()) {
        throw new NDEFException(
            String.format(
                "Expected language length %d but found %d",
                languageLength, LANGUAGE_CODE.length()));
      }
      for (int i = 0; i < LANGUAGE_CODE.length(); i++) {
        if (data.get() != LANGUAGE_CODE_BYTES[i]) {
          throw new NDEFException(
              String.format(
                  "Expected language %s, but found %s at position %d",
                  LANGUAGE_CODE, data.get(data.position() - 1), i));
        }
      }
      length -= (LANGUAGE_CODE.length() + 1);

      return length;
    }

    /** Custom Exception for NDEF parsing */
    public static class NDEFException extends Exception {
      public NDEFException() {}

      public NDEFException(String message) {
        super(message);
      }

      public NDEFException(String message, Throwable cause) {
        super(message, cause);
      }

      public NDEFException(Throwable cause) {
        super(cause);
      }
    }
  }

  /** Interface to allow multiple error handling options, to avoid program - lock in. */
  public interface ProgramSpecificErrorHandling {
    /**
     * Consume an error
     *
     * @param message user friendly message
     * @param e raw exception with stack trace
     */
    void consumeError(String message, Exception e);
    /**
     * Consume an error
     *
     * @param message message
     */
    void consumeError(String message);
    /**
     * Consume a warning
     *
     * @param message user friendly message
     * @param e raw exception with stack trace
     */
    void consumeWarning(String message, Exception e);
    /**
     * Consume a warning
     *
     * @param message message
     */
    void consumeWarning(String message);
  }

  private static class JavaErrors implements ProgramSpecificErrorHandling {

    /**
     * Consume an error
     *
     * @param message user friendly message
     * @param e raw exception with stack trace
     */
    @Override
    public void consumeError(String message, Exception e) {
      System.out.printf("Battery Tracking Error: %s \n", message);
      e.printStackTrace();
    }

    /**
     * Consume an error
     *
     * @param message message
     */
    @Override
    public void consumeError(String message) {
      System.out.printf("Battery Tracking Error: %s \n", message);
      (new RuntimeException(message)).printStackTrace();
    }

    /**
     * Consume a warning
     *
     * @param message user friendly message
     * @param e raw exception with stack trace
     */
    @Override
    public void consumeWarning(String message, Exception e) {
      System.out.printf("Battery Tracking Warning: %s \n", message);
      e.printStackTrace();
    }

    /**
     * Consume a warning
     *
     * @param message message
     */
    @Override
    public void consumeWarning(String message) {
      System.out.printf("Battery Tracking Warning: %s \n", message);
    }
  }
}
