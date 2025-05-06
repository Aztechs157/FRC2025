package frc.Robot.Commands.VisionCommands;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNull;

import java.lang.annotation.Target;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

import frc.robot.commands.VisionCommands.SetTargetTag;
import frc.robot.parsing.PositionDetails;
import frc.robot.parsing.PositionDetails.Position;
import frc.robot.subsystems.LEDSystem;
import frc.robot.subsystems.VisionSystem;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.Command;
public class SetTargetTagTest {

    public VisionSystemDecoy vision = new VisionSystemDecoy();
    public Position pos = Position.STAGE2;
    public PositionDetails details = new PositionDetails();
    
    public class VisionSystemDecoy extends VisionSystem {
        public List<Integer> ids;
        public int index = 0;
        public VisionSystemDecoy() {
            super(null);
        } 
        @Override
        public PhotonTrackedTarget findBestTargetReef() {
            return newTarget(ids.get(index++));
        }
        public void reset(List<Integer> ids) {
            this.ids = ids;
            index = 0;
        }
        @Override
        public void periodic() {

        }
    }

    public PhotonTrackedTarget newTarget(int id) {
        return new PhotonTrackedTarget(id, id, id, id, id, id, id, new Transform3d(), new Transform3d(), id, Arrays.asList(new TargetCorner(), new TargetCorner(), new TargetCorner(), new TargetCorner()), Arrays.asList(new TargetCorner(), new TargetCorner(), new TargetCorner(), new TargetCorner()));

    }

    public Command setTarget(boolean isLeft, Position location) {
        return new SetTargetTag(vision, isLeft, location, details);
    }

    @Test 
    public void test() {
        vision.reset(Arrays.asList(7, 7, 7, 7, 7, 7, 7));
        setTarget(true, Position.STAGE2).initialize();
        System.out.println(vision.desiredPose);

    }

    
}
