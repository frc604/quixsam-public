package com._604robotics.quixsam.mathematics;

public abstract class Interpolatable<T> {
  protected final T value;

  public Interpolatable(T value) {
    this.value = value;
  }

  public T get() {
    return this.value;
  }

  public abstract T interpolate(T endValue, double t);

  public T interpolate(Interpolatable<T> endValue, double t) {
    return interpolate(endValue.value, t);
  }
}