package com._604robotics.quixsam.mathematics;

import java.util.TreeMap;

public class DoubleInterpolatableTreeMap<T> {
  private final TreeMap<Double, Interpolatable<T>> treeMap =
      new TreeMap<Double, Interpolatable<T>>();

  public DoubleInterpolatableTreeMap() {}

  public void set(double time, Interpolatable<T> value) {
    treeMap.put(time, value);
  }

  public void clear() {
    treeMap.clear();
  }

  public T get(double time) {
    if (treeMap.isEmpty()) return null;

    var value = treeMap.get(time);
    if (value != null) return value.get();

    // Returns the entry with the least key that is greater than or equal to the time.
    var topBound = treeMap.ceilingEntry(time);
    // Returns the entry with the greatest key that is less than or equal to the time.
    var bottomBound = treeMap.floorEntry(time);

    if (topBound == null) {
      return bottomBound.getValue().get();
    } else if (bottomBound == null) {
      return topBound.getValue().get();
    } else {
      return bottomBound
          .getValue()
          .interpolate(
              topBound.getValue(),
              (time - bottomBound.getKey()) / (topBound.getKey() - bottomBound.getKey()));
    }
  }
}