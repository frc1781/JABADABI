package frc.robot;

public enum Robots {
  DE_RONNA, 
  SAVITAR, 
  AVA, 
  RALPH,
  NOBODY;

  String asString() {
    switch (this) {
      case DE_RONNA:
        return "deRonna";
      case SAVITAR:
        return "savitara";
      case AVA:
        return "ava";
      case RALPH:
        return "ralph";
      default:
        return "nobody";
    }
  }
}  
