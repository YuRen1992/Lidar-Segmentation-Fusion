/*
 * point_type implementation
 *
 * Copyright (C) by the 3DTK contributors
 *
 * Released under the GPL version 3.
 *
 */

/**
 * @file
 * @brief Representation of a 3D point type
 * @author Jan Elsberg. Jacobs University Bremen gGmbH, Germany. 
 */

#include "point_type.h"

//#include "scan.h"

#include <string>
using std::string;
#include <iostream>
using std::cout;
using std::cerr;
using std::endl;

#include <fstream>
#include <string.h>

#include <stdexcept>
using std::runtime_error;

PointType::PointType() {
  types = USE_NONE;
  pointdim = 3;
  // choose height per default  
  dimensionmap[1] = dimensionmap[2] = dimensionmap[3] = 1;
  dimensionmap[4] = dimensionmap[5] = dimensionmap[6] = 1; 
  dimensionmap[7] = dimensionmap[8] = dimensionmap[0] = 1;
  dimensionmap[9] = 1;  // height  
}

PointType::PointType(unsigned int _types) : types(_types) {
  // choose height per default  
  dimensionmap[1] = dimensionmap[2] = dimensionmap[3] = 1;
  dimensionmap[4] = dimensionmap[5] = dimensionmap[6] = 1; 
  dimensionmap[7] = dimensionmap[8] = dimensionmap[0] = 1; 
  dimensionmap[9] = 1;  // height 

  pointdim = 3;
  if (types & PointType::USE_REFLECTANCE) dimensionmap[1] = pointdim++;  
  if (types & PointType::USE_NORMAL) {
    pointdim += 3;
    dimensionmap[2] = pointdim;
  }
  if (types & PointType::USE_TEMPERATURE) dimensionmap[3] = pointdim++;  
  if (types & PointType::USE_AMPLITUDE)   dimensionmap[4] = pointdim++;  
  if (types & PointType::USE_DEVIATION)   dimensionmap[5] = pointdim++;  
  if (types & PointType::USE_TYPE)        dimensionmap[6] = pointdim++; 
  if (types & PointType::USE_COLOR)       dimensionmap[7] = pointdim++; 
  if (types & PointType::USE_TIME)        dimensionmap[8] = pointdim++; 
  if (types & PointType::USE_INDEX)       dimensionmap[9] = pointdim++; 
}

bool PointType::hasReflectance() {
  return hasType(USE_REFLECTANCE); 
}
bool PointType::hasNormal() {
  return hasType(USE_NORMAL); 
}
bool PointType::hasTemperature() {
  return hasType(USE_TEMPERATURE); 
}
bool PointType::hasAmplitude() {
  return hasType(USE_AMPLITUDE); 
}
bool PointType::hasDeviation() {
  return hasType(USE_DEVIATION); 
}
bool PointType::hasType() {
  return hasType(USE_TYPE); 
}
bool PointType::hasColor() {
  return hasType(USE_COLOR); 
}
bool PointType::hasTime() {
  return hasType(USE_TIME); 
}

bool PointType::hasIndex() {
  return hasType(USE_INDEX); 
}

unsigned int PointType::getReflectance() {
  return dimensionmap[1];
}

unsigned int PointType::getTemperature() {
  return dimensionmap[3];
}

unsigned int PointType::getAmplitude() {
  return dimensionmap[4];
}

unsigned int PointType::getDeviation() {
  return dimensionmap[5];
}

unsigned int PointType::getTime() {
  return dimensionmap[8];
}

unsigned int PointType::getIndex() {
  return dimensionmap[9];
}

unsigned int PointType::getType() {
  return dimensionmap[6];
}

unsigned int PointType::getColor() {
  return dimensionmap[7];
}

unsigned int PointType::getType(unsigned int type) {
  if (type == USE_NONE ) {
    return dimensionmap[0];
  } else if (type == USE_HEIGHT) {
    return dimensionmap[0];
  } else if (type == USE_REFLECTANCE) {
    return dimensionmap[1];
  } else if (type == USE_NORMAL) {
    return dimensionmap[2];
  } else if (type == USE_TEMPERATURE) {
    return dimensionmap[3];
  } else if (type == USE_AMPLITUDE) {
    return dimensionmap[4];
  } else if (type == USE_DEVIATION) {
    return dimensionmap[5];
  } else if (type == USE_TYPE) {
    return dimensionmap[6];
  } else if (type == USE_COLOR) {
    return dimensionmap[7];
  } else if (type == USE_TIME) {
    return dimensionmap[8];
  } else if (type == USE_INDEX) {
    return dimensionmap[9];
  } else {
    return 0;
  }
}


unsigned int PointType::getPointDim() { return pointdim; }

PointType PointType::deserialize(std::ifstream &f) {
  unsigned int types;
  f.read(reinterpret_cast<char*>(&types), sizeof(unsigned int));
  return PointType(types);
}

void PointType::serialize(std::ofstream &f) {
  f.write(reinterpret_cast<char*>(&types), sizeof(unsigned int));
}

unsigned int PointType::toFlags() const { return types; } 

bool PointType::hasType(unsigned int type) {
  return types & type;
}


const unsigned int PointType::USE_NONE = 0;
const unsigned int PointType::USE_REFLECTANCE = 1;
const unsigned int PointType::USE_NORMAL = 2;
const unsigned int PointType::USE_TEMPERATURE = 4;
const unsigned int PointType::USE_AMPLITUDE = 8;
const unsigned int PointType::USE_DEVIATION = 16;
const unsigned int PointType::USE_HEIGHT = 32;
const unsigned int PointType::USE_TYPE = 64;
const unsigned int PointType::USE_COLOR = 128;
const unsigned int PointType::USE_TIME = 256;
const unsigned int PointType::USE_INDEX = 512;


//void PointType::useScan(Scan* scan)
//{
//  // clear pointers first
//  m_xyz = 0; m_rgb = 0; m_reflectance = 0; m_temperature = 0; m_amplitude = 0; m_type = 0; m_deviation = 0;
//  
//  // collectively load data to avoid unneccessary loading times due
//  // to split get("") calls
//  unsigned int types = DATA_XYZ;
//  if(hasColor()) types |= DATA_RGB;
//  if(hasReflectance()) types |= DATA_REFLECTANCE;
//  if(hasTemperature()) types |= DATA_TEMPERATURE;
//  if(hasAmplitude()) types |= DATA_AMPLITUDE;
//  if(hasType()) types |= DATA_TYPE;
//  if(hasDeviation()) types |= DATA_DEVIATION;
//  scan->get(types);
//  
//  // access data
//  try {
//    m_xyz = new DataXYZ(scan->get("xyz"));
//    if(hasColor()) m_rgb = new DataRGB(scan->get("rgb"));
//    if(hasReflectance()) m_reflectance = new
//                           DataReflectance(scan->get("reflectance"));
//    if(hasTemperature()) m_temperature = new
//                           DataTemperature(scan->get("temperature"));
//    if(hasAmplitude()) m_amplitude = new DataAmplitude(scan->get("amplitude"));
//    if(hasType()) m_type = new DataType(scan->get("type"));
//    if(hasDeviation()) m_deviation = new DataDeviation(scan->get("deviation"));
//    
//    // check if data is available, otherwise reset pointer
//    // to indicate that the scan doesn't prove this value
//    if (m_rgb && !m_rgb->valid()) {
//      delete m_rgb; m_rgb = 0;
//    }
//    if (m_reflectance && !m_reflectance->valid()) {
//      delete m_reflectance; m_reflectance = 0;
//    }
//    if (m_temperature && !m_temperature->valid()) {
//      delete m_temperature; m_temperature = 0;
//    }
//    if (m_amplitude && !m_amplitude->valid()) {
//      delete m_amplitude; m_amplitude = 0;
//    }
//    if (m_type && !m_type->valid()) {
//      delete m_type; m_type = 0;
//    }
//    if (m_deviation && !m_deviation->valid()) {
//      delete m_deviation; m_deviation = 0;
//    }
//  } catch(runtime_error& e) {
//    // unlock everything again
//    clearScan();
//    throw e;
//  }
//}

void PointType::clearScan()
{
  // unlock data access
  if (m_xyz) delete m_xyz;
  if (hasColor() && m_rgb) delete m_rgb;
  if (hasReflectance() && m_reflectance) delete m_reflectance;
  if (hasTemperature() && m_temperature) delete m_temperature;
  if (hasAmplitude() && m_amplitude) delete m_amplitude;
  if (hasType() && m_type) delete m_type;
  if (hasDeviation() && m_deviation) delete m_deviation;
  
  // TODO: scan->clear() on all of these types
}


//unsigned int PointType::getScanSize(Scan* scan)
//{
//  return scan->size<DataXYZ>("xyz");
//}
