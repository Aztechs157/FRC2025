// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.parsing;

import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Dictionary;
import java.util.Hashtable;
import java.util.Iterator;
import java.util.List;

import edu.wpi.first.wpilibj.Filesystem;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;

/** Add your docs here. */
public class PositionDetails {
    private final String betaJSONPath = "/beta_position_details.json";
    private final String alphaJSONPath = "/alpha_position_details.json";
    public Stage[] corals = new Stage[5];

    public enum Position {
        STAGE1(1),
        STAGE2(2),
        STAGE3(3),
        STAGE4(4),
        ALGAE1(6), // low
        ALGAE2(7), // high
        CORALSTATION(0),
        BASE(10);

        public int stageNum;

        private Position(int stageNum) {
            this.stageNum = stageNum;
        }
    };

    public Dictionary<Integer, ReefTag> reefTags = new Hashtable<>();

    public class Stage {
        public double leftOffset;
        public double rightOffset;
        public double elevatorPos;
        public double elbowPos;
        public double wristPos;
        public double depthOffset;

        public Stage(JsonNode stageJSON) {
            // System.out.println(stageJSON.toPrettyString());
            this.leftOffset = stageJSON.get("horizontalOffsets").get("left").asDouble();
            this.rightOffset = stageJSON.get("horizontalOffsets").get("right").asDouble();
            this.depthOffset = stageJSON.get("depthOffset").asDouble();
            this.elevatorPos = stageJSON.get("systemPositions").get("elevator").asDouble();
            this.elbowPos = stageJSON.get("systemPositions").get("elbow").asDouble();
            this.wristPos = stageJSON.get("systemPositions").get("wrist").asDouble();
        }
    }

    public PositionDetails(boolean isBeta) { // TODO bug here parsing int in line 49
        File file = new File(Filesystem.getDeployDirectory().toPath() + (isBeta ? betaJSONPath : alphaJSONPath));
        ObjectMapper objectMapper = new ObjectMapper();

        try {

            JsonNode json = objectMapper.readTree(file);

            for (Iterator<String> i = json.get("reef").fieldNames(); i.hasNext();) {
                String currentTag = i.next();

                // TODO check if "currentTag" starts with "tag", as it is currently trying to
                // read "coral" as a number - Katie
                if (currentTag.contains("tag")) {
                    int tagID = Integer.parseInt(currentTag.substring(3));
                    reefTags.put(tagID, new ReefTag(json.get("reef").get(currentTag), tagID));
                }
            }

            List<Stage> coralList = new ArrayList<Stage>(5);
            coralList.add(new Stage(json.get("coralStation")));
            for (int i = 1; i <= 4; i++) {
                coralList.add(new Stage(json.get("reef").get("coral").get("stage" + i)));
            }

            coralList.toArray(corals);

        } catch (IOException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        }
    }

    public PositionDetails() {
        this(true);
    }

    /**
     * Get elevator position for the algae
     * 
     * @param tagID - red oriented tag IDs ie: blue 17 -> red 6
     * @return a double representing the elevator position
     */
    public double getElevatorPosAtAlgae(int tagID) {
        return reefTags.get(tagID).getElevatorPosAtAlgae();
    }

    /**
     * Get elbow position for the algae
     * 
     * @param tagID - red oriented tag IDs ie: blue 17 -> red 6
     * @return a double representing the elbow position
     */
    public double getElbowPosAtAlgae(int tagID) {
        return reefTags.get(tagID).getElbowPosAtAlgae();
    }

    /**
     * Get wrist position for the algae
     * 
     * @param tagID - red oriented tag IDs ie: blue 17 -> red 6
     * @return a double representing the wrist position
     */
    public double getWristPosAtAlgae(int tagID) {
        return reefTags.get(tagID).getWristPosAtAlgae();
    }

    /**
     * Get elevator position for the specified stage
     * 
     * @param stage - one-indexed ie: 1,2,3,4
     * @return a double representing the elevator position
     */
    public double getElevatorPosAtStage(int stage) {
        return corals[stage].elevatorPos;
    }

    /**
     * Get elbow position for the specified stage
     * 
     * @param stage - one-indexed ie: 1,2,3,4
     * @return a double representing the elbow position
     */
    public double getElbowPosAtStage(int stage) {
        return corals[stage].elbowPos;
    }

    /**
     * Get wrist position for the specified stage
     * 
     * @param stage - one-indexed ie: 1,2,3,4
     * @return a double representing the wrist position
     */
    public double getWristPosAtStage(int stage) {
        return corals[stage].wristPos;
    }

    /**
     * Get left offset for the specified stage
     * 
     * @param stage - one-indexed ie: 1,2,3,4
     * @return a double representing the left offset
     */
    public double getLeftOffsetAtStage(int stage) {
        return corals[stage].leftOffset;
    }

    /**
     * Get right offset for the specified stage
     * 
     * @param stage - one-indexed ie: 1,2,3,4
     * @return a double representing the right offset
     */
    public double getRightOffsetAtStage(int stage) {
        return corals[stage].rightOffset;
    }

    /**
     * Get right offset for the specified stage
     * 
     * @param stage - one-indexed ie: 1,2,3,4
     * @return a double representing the right offset
     */
    public double getDepthOffsetAtStage(int stage) {
        return corals[stage].depthOffset;
    }

    /**
     * Get elevator position for the specified stage
     * 
     * @param stage - one-indexed ie: 1,2,3,4 TODO update
     * @return a double representing the elevator position
     */
    public double getElevatorPos(Position pos) {
        switch (pos) {
            case STAGE1, STAGE2, STAGE3, STAGE4, CORALSTATION:
                return getElevatorPosAtStage(pos.stageNum);
            case ALGAE1, ALGAE2:
                return getElevatorPosAtAlgae(pos.stageNum);
            case BASE:
                return 0;
            default:
                return 0;
        }
    }

    /**
     * Get elbow position for the specified stage
     * 
     * @param stage - one-indexed ie: 1,2,3,4 TODO update
     * @return a double representing the elbow position
     */
    public double getElbowPos(Position pos) {
        switch (pos) {
            case STAGE1, STAGE2, STAGE3, STAGE4, CORALSTATION:
                return getElbowPosAtStage(pos.stageNum);
            case ALGAE1, ALGAE2:
                return getElbowPosAtAlgae(pos.stageNum);
            default:
                return 0;
        }
    }

    /**
     * Get Wrist position for the specified stage
     * 
     * @param stage - one-indexed ie: 1,2,3,4 TODO update
     * @return a double representing the Wrist position
     */
    public double getWristPos(Position pos) {
        switch (pos) {
            case STAGE1, STAGE2, STAGE3, STAGE4, CORALSTATION:
                return getWristPosAtStage(pos.stageNum);
            case ALGAE1, ALGAE2:
                return getWristPosAtAlgae(pos.stageNum);
            default:
                return 0;
        }
    }
}
