
import { Badge } from "@/components/ui/badge";

interface RobotStatusBadgeProps {
  status: string;
}

export function RobotStatusBadge({ status }: RobotStatusBadgeProps) {
  switch (status) {
    case 'online':
      return (
        <Badge variant="outline" className="bg-robot-online/10 text-robot-online border-robot-online">
          <span className="w-2 h-2 rounded-full bg-robot-online mr-1.5 animate-pulse"></span>
          Online
        </Badge>
      );
    case 'warning':
      return (
        <Badge variant="outline" className="bg-robot-warning/10 text-robot-warning border-robot-warning">
          <span className="w-2 h-2 rounded-full bg-robot-warning mr-1.5"></span>
          Warning
        </Badge>
      );
    case 'offline':
    default:
      return (
        <Badge variant="outline" className="bg-robot-offline/10 text-robot-offline border-robot-offline">
          <span className="w-2 h-2 rounded-full bg-robot-offline mr-1.5"></span>
          Offline
        </Badge>
      );
  }
}
