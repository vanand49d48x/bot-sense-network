
import { RobotStatusCard } from "./RobotStatusCard";
import { Robot } from "@/hooks/useRobots";

interface RobotStatusGridProps {
  robots: Robot[];
}

export function RobotStatusGrid({ robots }: RobotStatusGridProps) {
  return (
    <div className="mt-6">
      <h2 className="text-lg font-semibold mb-4">Robot Status</h2>
      <div className="grid gap-6 md:grid-cols-2 lg:grid-cols-3">
        {robots.map((robot) => (
          <RobotStatusCard key={robot.id} robot={robot} />
        ))}
      </div>
    </div>
  );
}
