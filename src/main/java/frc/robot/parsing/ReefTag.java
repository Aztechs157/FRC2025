// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.parsing;

import com.fasterxml.jackson.databind.JsonNode;

/** Add your docs here. */
public class ReefTag {
    public int tagID;
    public String face;
    public double algaeVerticalPos;
    public double elevatorPos;
    public double elbowPos;
    public double wristPos;

    public ReefTag(JsonNode tagJSON, int tagID) {
        this.tagID = tagID; // TODO: Try to get tag ID from tagJSON
        this.face = tagJSON.get("face").asText();
        this.algaeVerticalPos = tagJSON.get("algaeVerticalPos").asDouble();
        this.elevatorPos = tagJSON.get("systemPositions").get("elevator").asDouble();
        this.elbowPos = tagJSON.get("systemPositions").get("elbow").asDouble();
        this.wristPos = tagJSON.get("systemPositions").get("wrist").asDouble();
    }

    /**
     * Get elevator position for the algae
     * 
     * @return a double representing the elevator position
     */
    public double getElevatorPosAtAlgae() {
        return elevatorPos;
    }

    /**
     * Get elbow position for the algae
     * 
     * @return a double representing the elbow position
     */
    public double getElbowPosAtAlgae() {
        return elbowPos;
    }

    /**
     * Get wrist position for the algae
     * 
     * @return a double representing the wrist position
     */
    public double getWristPosAtAlgae() {
        return wristPos;
    }
}