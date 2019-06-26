#include "ublox.h"

#include <time.h>

#define DEG2RAD (3.14159 / 180.0)
//#define DBG(...) printf(__VA_ARGS__)
#define DBG(...)
#ifndef M_PI
#define M_PI 3.14159
#endif

UBLOX *gnss_Ptr;

//A C style callback
void cb(uint8_t byte)
{
  gnss_Ptr->read_cb(byte);
}

UBLOX::UBLOX() {}

//Look for a GNSS receiver, and if found, change its settings
void UBLOX::init(UART *uart)
{
  gnss_Ptr = this;
  // Reset message parser
  buffer_head_ = 0;
  parse_state_ = START;
  message_class_ = 0;
  message_type_ = 0;
  length_ = 0;
  ck_a_ = 0;
  ck_b_ = 0;

  // Find the right baudrate
  looking_for_nmea_ = true;
  serial_ = uart;


  //serial_->set_mode(115200, UART::MODE_8N1);
  serial_->set_mode(57600, UART::MODE_8N1);
  serial_->register_rx_callback(cb);

  if (!detect_baudrate())
    return;

  // Otherwise, Configure the GNSS receiver
  //set_baudrate(115200);
  //set_baudrate(57600);
  set_dynamic_mode();
  set_nav_rate(100);
  enable_message(CLASS_NAV, NAV_PVT, 1);
  enable_message(CLASS_NAV, NAV_POSECEF, 1);
  enable_message(CLASS_NAV, NAV_VELECEF, 1);

  //Zeroing the data
  this->nav_message_ = {};
}

//Attempt to detect the baudrate by trying a rate, waiting 1 s, and
//seeing if any messages are recieved.
//Attempted baudrates and the order in which they are tried are defined in ublox.h, in the array 'baudrates'
//As a consequence of the changing baudrates, the sensor is treated as
//disconnected until the procedure finishes
bool UBLOX::detect_baudrate()
{
  current_baudrate_ = 0;
  for (uint32_t i = 0; i < sizeof(baudrates)/sizeof(uint32_t); i++)
  {
    DBG("Trying %d baudrate\n", baudrates[i]);
    serial_->set_mode(baudrates[i], UART::MODE_8N1);
    uint32_t timeout_ms(1000);
    uint32_t start_ms = millis();
    uint32_t now_ms = millis();
    //TREYDO should got_message_ be set to false here?
    got_message_ = false;
    while (now_ms < start_ms + timeout_ms)
    {
      now_ms = millis();
      if (got_message_)
      {
        DBG("Found UBLOX at %d baud\n", baudrates[i]);
        current_baudrate_ = baudrates[i];
        break;
      }
    }
    if (current_baudrate_ != 0)
      break;
  }
  return got_message_;
}

bool UBLOX::send_message(uint8_t msg_class, uint8_t msg_id, UBX_message_t &message, uint16_t len)
{
  // First, calculate the checksum
  uint8_t ck_a, ck_b;
  //TREYDO pass by reference, with ints?
  calculate_checksum(msg_class, msg_id, len, message, ck_a, ck_b);

  // Send message
  serial_->put_byte(START_BYTE_1);
  serial_->put_byte(START_BYTE_2);
  serial_->put_byte(msg_class);
  serial_->put_byte(msg_id);
  serial_->put_byte(len & 0xFF);
  serial_->put_byte((len >> 8) & 0xFF);
  serial_->write(message.buffer, len);
  serial_->put_byte(ck_a);
  serial_->put_byte(ck_b);
  return true;
}

//Set the baudrate and other settings on the GNSS receiver
//This requires the flight controller to have detected the correct, current baud rate
//This also changes the flight controller's baud rate to match the GNSS reciever
void UBLOX::set_baudrate(const uint32_t baudrate)
{
  DBG("Setting baudrate to %d\n", baudrate);
  // Now that we have the right baudrate, let's configure the thing
  memset(&out_message_, 0, sizeof(CFG_PRT_t));
  out_message_.CFG_PRT.portID = CFG_PRT_t::PORT_UART1;
  out_message_.CFG_PRT.baudrate = baudrate;
  out_message_.CFG_PRT.inProtoMask = CFG_PRT_t::IN_UBX | CFG_PRT_t::IN_NMEA | CFG_PRT_t::IN_RTCM;
  out_message_.CFG_PRT.outProtoMask = CFG_PRT_t::OUT_UBX | CFG_PRT_t::OUT_NMEA;
  out_message_.CFG_PRT.mode = CFG_PRT_t::CHARLEN_8BIT | CFG_PRT_t::PARITY_NONE | CFG_PRT_t::STOP_BITS_1;
  out_message_.CFG_PRT.flags = 0;
  send_message(CLASS_CFG, CFG_PRT, out_message_, sizeof(CFG_PRT_t));
  delayMicroseconds(10000);
  serial_->set_mode(baudrate, UART::MODE_8N1);
  current_baudrate_ = baudrate;
}

//Checks if there is a GNSS reciever present, which is determined by checking if
//it has ever recieved a valid message
bool UBLOX::present()
{
  return got_message_;
}

//Set the dynamic mode to airborne, with a max acceleration of 4G
void UBLOX::set_dynamic_mode()
{
  memset(&out_message_, 0, sizeof(CFG_NAV5_t));
  out_message_.CFG_NAV5.mask = CFG_NAV5_t::MASK_DYN;
  out_message_.CFG_NAV5.dynModel = CFG_NAV5_t::DYNMODE_AIRBORNE_4G;
  DBG("Setting dynamic mode\n");
  send_message(CLASS_CFG, CFG_PRT, out_message_, sizeof(CFG_NAV5_t));
}

//Set the frequency of nav messages, defined as the period in ms
void UBLOX::set_nav_rate(uint8_t period_ms)
{
  memset(&out_message_, 0, sizeof(CFG_RATE_t));
  out_message_.CFG_RATE.measRate = period_ms;
  out_message_.CFG_RATE.navRate = 1;
  out_message_.CFG_RATE.timeRef = CFG_RATE_t::TIME_REF_GPS;
  DBG("Setting nav rate to %d\n", period_ms);
  send_message(CLASS_CFG, CFG_RATE, out_message_, sizeof(CFG_RATE_t));
}

void UBLOX::enable_message(uint8_t msg_cls, uint8_t msg_id, uint8_t rate)
{
  memset(&out_message_, 0, sizeof(CFG_MSG_t));
  out_message_.CFG_MSG.msgClass = msg_cls;
  out_message_.CFG_MSG.msgID = msg_id;
  out_message_.CFG_MSG.rate = rate;
  DBG("Requesting %x:%x message at %d hz\n", msg_cls, msg_id, rate);
  send_message(CLASS_CFG, CFG_MSG, out_message_, sizeof(CFG_MSG_t));
}


void UBLOX::read_cb(uint8_t byte)
{
  uint64_t time_recieved = micros();
  // Look for a valid NMEA packet (do this at the beginning in case
  // UBX was disabled for some reason) and during autobaud
  // detection
  if (looking_for_nmea_)
  {
    if (byte == NMEA_START_BYTE2 && prev_byte_ == NMEA_START_BYTE1)
    {
      got_message_ = true;
      looking_for_nmea_ = false;
    }
  }

  // handle the UBX packet
  switch (parse_state_)
  {
  case START:
    if (byte == START_BYTE_2 && prev_byte_ == START_BYTE_1)
    {
      looking_for_nmea_ = false;
      buffer_head_ = 0;
      parse_state_ = GOT_START_FRAME;
      message_class_ = 0;
      message_type_ = 0;
      length_ = 0;
      ck_a_ = 0;
      ck_b_ = 0;
      got_message_ = true;
    }
    break;
  case GOT_START_FRAME:
    message_class_ = byte;
    parse_state_ = GOT_CLASS;
    break;

  case GOT_CLASS:
    message_type_ = byte;
    parse_state_ = GOT_MSG_ID;
    break;
  case GOT_MSG_ID:
    length_ = byte;
    parse_state_ = GOT_LENGTH1;
    break;
  case GOT_LENGTH1:
    length_ |= static_cast<uint16_t>(byte) << 8;
    parse_state_ = GOT_LENGTH2;
    if (length_ > UBLOX_BUFFER_SIZE)
    {
      num_errors_++;
      parse_state_ = START;
      return;
    }
    break;
  case GOT_LENGTH2:
    if (buffer_head_ < length_)
    {
      // push the byte onto the data buffer
      in_message_.buffer[buffer_head_] = byte;
      if (buffer_head_ == length_-1)
      {
        parse_state_ = GOT_PAYLOAD;
      }
      buffer_head_++;
    }
    break;
  case GOT_PAYLOAD:
    ck_a_ = byte;
    parse_state_ = GOT_CK_A;
    break;
  case GOT_CK_A:
    ck_b_ = byte;
    parse_state_ = GOT_CK_B;
    break;
  default:
    num_errors_++;
    break;
  }

  // If we have a complete packet, then try to parse it
  if (parse_state_ == GOT_CK_B)
  {
    if (decode_message())
    {
      parse_state_ = START;
      if (message_class_ == CLASS_NAV && message_type_ == NAV_PVT)
        last_pvt_timestamp = time_recieved;
    }
    else
    {
      // indicate error if it didn't work
      num_errors_++;
      DBG("failed to parse message\n");
      parse_state_ = START;
    }
  }

  prev_byte_ = byte;
}
//convert a time struct to unix time
//dumb leap years
//This function will break in 2100, because it doesn't take into accoutn that 2100 is not a leap year
uint64_t convert_to_unix(UBLOX::GNSS_TIME_T time)
{
  uint32_t day_s = 24*60*60; //seconds per day
  uint32_t year_s = 365*day_s; //seconds per non-leap day
  uint32_t elapsed_years = time.year-1970;
  uint32_t elapsed_leap_days = (elapsed_years+1)/4;
  if (time.year % 4 == 0) //If currently in a leap year
    if (time.month>=3) //If past feb 29
      elapsed_leap_days++;
  uint32_t elapsed_days;//Days past in the year
  switch (time.month)
  {
  case 1:
    elapsed_days = 0;
    break;
  case 2:
    elapsed_days = 31;
    break;
  case 3:
    //Ignore leap days, because it is accounted for above
    elapsed_days = 31+28;
    break;
  case 4:
    elapsed_days = 31+28+31;
    break;
  case 5:
    elapsed_days = 31+28+31+30;
    break;
  case 6:
    elapsed_days = 31+28+31+30+31;
    break;
  case 7:
    elapsed_days = 31+28+31+30+31+30;
    break;
  case 8:
    elapsed_days = 31+28+31+30+31+30+31;
    break;
  case 9:
    elapsed_days = 31+28+31+30+31+30+31+31;
    break;
  case 10:
    elapsed_days = 31+28+31+30+31+30+31+31+30;
    break;
  case 11:
    elapsed_days = 31+28+31+30+31+30+31+31+30+31;
    break;
  case 12:
    elapsed_days = 31+28+31+30+31+30+31+31+30+31+30;
    break;
  default://This should never be reached, because there are 12 month / year
    elapsed_days = 0;
  }
  elapsed_days += time.day-1; //Minus 1 because the day is not yet complete
  return elapsed_years * year_s + (elapsed_days+elapsed_leap_days)*day_s + (time.hour*60+time.min)*60 + time.sec;
}
/* Tells if new data is available
 * Only returns true if new data has been recieved, and if all three sources match time of week.
 * This is because if the times of week do not match, new data is still being recieved,
 * and attempting to read could result in data from different times.
 */
bool UBLOX::new_data()
{
  return this->new_data_;
//   return this->new_data_
//       && (this->nav_message_.iTOW == this->pos_ecef_.iTOW)
//       && (this->nav_message_.iTOW == this->vel_ecef_.iTOW);
}
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmissing-field-initializers" //Ignore warning about leaving struct fields blank
UBLOX::GNSSPVT UBLOX::read()
{
  GNSSPVT data = {};
  data.time_of_week = this->nav_message_.iTOW;
  data.time = convert_to_unix(this->nav_message_.time);
  data.nanos = this->nav_message_.time.nano;
  data.lat = this->nav_message_.lat;
  data.lon = this->nav_message_.lon;
  data.height = this->nav_message_.height;
  data.vel_n = this->nav_message_.velN;
  data.vel_e = this->nav_message_.velE;
  data.vel_d = this->nav_message_.velD;
  data.h_acc = this->nav_message_.hAcc;
  data.v_acc = this->nav_message_.vAcc;
  data.rosflight_timestamp = this->last_pvt_timestamp;

  this->new_data_=false;

  return data;
}

UBLOX::GNSSPosECEF UBLOX::read_pos_ecef()
{
  GNSSPosECEF pos = {};
  pos.x = this->pos_ecef_.ecefX;
  pos.y = this->pos_ecef_.ecefY;
  pos.z = this->pos_ecef_.ecefZ;
  pos.time_of_week = this->pos_ecef_.iTOW;
  pos.p_acc = this->pos_ecef_.pAcc;
  return pos;
}

UBLOX::GNSSVelECEF UBLOX::read_vel_ecef()
{
  UBLOX::GNSSVelECEF vel = {};
  vel.vx = this->vel_ecef_.ecefVX;
  vel.vy = this->vel_ecef_.ecefVY;
  vel.vz = this->vel_ecef_.ecefVZ;
  vel.time_of_week = this->vel_ecef_.iTOW;
  vel.s_acc = this->vel_ecef_.sAcc;
  return vel;
}

const UBLOX::NAV_PVT_t &UBLOX::read_raw()
{
  return this->nav_message_;
}
#pragma GCC diagnostic pop //End ignore blank struct initalizers

bool UBLOX::decode_message()
{
  // First, check the checksum
  uint8_t ck_a, ck_b;
  calculate_checksum(message_class_, message_type_, length_, in_message_, ck_a, ck_b);
  if (ck_a != ck_a_ || ck_b != ck_b_)
    return false;

  num_messages_received_++;
  DBG("recieved message %d: ", num_messages_received_);

  // Parse the payload
  switch (message_class_)
  {
  case CLASS_ACK:
    DBG("ACK_");
    switch (message_type_)
    {
    case ACK_ACK:
      got_ack_ = true;
      DBG("ACK\n");
      break;
    case ACK_NACK:
      got_nack_ = true;
      DBG("NACK\n");
      break;
    default:
      DBG("%d\n", message_type_);
      break;
    }
    break;


  case CLASS_CFG:
    DBG("CFG_");
    switch (message_type_)
    {
    default:
      DBG("%d\n", message_type_);
      break;
    }
    break;

  case CLASS_NAV:
    DBG("NAV_");
    switch (message_type_)
    {
    case NAV_PVT:
      new_data_ = true;
      nav_message_ = in_message_.NAV_PVT;
      DBG("PVT\n");
      break;
    case NAV_POSECEF:
      new_data_ = true;
      pos_ecef_ = in_message_.NAV_POSECEF;
      DBG("POSECEF\n");
      break;
    case NAV_VELECEF:
      new_data_ = true;
      vel_ecef_ = in_message_.NAV_VELECEF;
      DBG("NAVELECEF\n");
      break;
    default:
      DBG("%d\n", message_type_);
      break;
    }
    break;
  default:
    DBG("%d_%d\n", message_class_, message_type_);
    break;
  }
  return true;
}

void UBLOX::convert_data()
{
  lla_[0] = static_cast<double>(nav_message_.lat) * 1e-7L;
  lla_[1] = static_cast<double>(nav_message_.lon) * 1e-7L;
  lla_[2] = nav_message_.height * 1e-3;

  vel_[0] = nav_message_.velN * 1e-3;
  vel_[1] = nav_message_.velE * 1e-3;
  vel_[2] = nav_message_.velD * 1e-3;
  time_ = convert_to_unix(nav_message_.time);
}

void UBLOX::calculate_checksum(const uint8_t msg_cls, const uint8_t msg_id, const uint16_t len,
                               const UBX_message_t payload, uint8_t &ck_a, uint8_t &ck_b) const
{
  ck_a = ck_b = 0;

  // Add in class
  ck_a += msg_cls;
  ck_b += ck_a;

  // Id
  ck_a += msg_id;
  ck_b += ck_a;

  // Length
  ck_a += len & 0xFF;
  ck_b += ck_a;
  ck_a += (len >> 8) & 0xFF;
  ck_b += ck_a;

  // Payload
  for (int i = 0; i < len; i ++)
  {
    ck_a += payload.buffer[i];
    ck_b += ck_a;
  }
}


