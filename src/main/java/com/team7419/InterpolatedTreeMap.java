package com.team7419;

import java.util.TreeMap;

public class InterpolatedTreeMap extends TreeMap<Double, Double> {

  @Override
  public Double get(Object value) {
    double doubleValue = (double) value;
    Double upper = ceilingKey(doubleValue);
    Double lower = floorKey(doubleValue);
    if (upper == null && lower == null) return null;
    else if (upper == null) return super.get(lower);
    else if (lower == null) return super.get(upper);
    if (upper.doubleValue() == lower.doubleValue()) return super.get(upper);
    double diffPercent = (doubleValue - lower) / (upper - lower);
    return diffPercent * (super.get(upper) - super.get(lower)) + super.get(lower);
  }
}
