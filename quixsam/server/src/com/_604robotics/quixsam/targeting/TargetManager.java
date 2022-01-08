package com._604robotics.quixsam.targeting;

import java.util.HashMap;

public class TargetManager {
  private HashMap<Integer, Landmark> landmarkMap;

  private static TargetManager single_instance = null;

  public static TargetManager getInstance() {
    if (single_instance == null) single_instance = new TargetManager();

    return single_instance;
  }

  public TargetManager(Target... targets) {
    int currentID = 0;
    for (Target target : targets) {
      for (Landmark landmark : target.getLandmarks()) {
        landmarkMap.put(currentID, landmark);
        currentID++;
      }
    }
  }


}
