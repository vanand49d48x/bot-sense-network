
import { useState } from "react";
import { Button } from "@/components/ui/button";
import {
  Select,
  SelectContent,
  SelectItem,
  SelectTrigger,
  SelectValue,
} from "@/components/ui/select";
import { formatDistance } from "date-fns";
import {
  ChevronLeft,
  ChevronRight,
  ChevronsLeft,
  ChevronsRight,
} from "lucide-react";

type Telemetry = {
  id: string;
  timestamp: string;
  battery_level: number;
  temperature: number;
  location?: {
    latitude: number;
    longitude: number;
  };
};

interface TelemetryTableProps {
  telemetry: Telemetry[];
  loading: boolean;
}

export function TelemetryTable({ telemetry, loading }: TelemetryTableProps) {
  const [recordsPerPage, setRecordsPerPage] = useState("10");
  const [currentPage, setCurrentPage] = useState(1);

  // Calculate pagination
  const recordsLimit = parseInt(recordsPerPage);
  const totalPages = Math.ceil(telemetry.length / recordsLimit);
  const startIndex = (currentPage - 1) * recordsLimit;
  const endIndex = startIndex + recordsLimit;
  const currentRecords = telemetry.slice(startIndex, endIndex);

  // Page navigation
  const goToPage = (page: number) => {
    setCurrentPage(Math.max(1, Math.min(page, totalPages)));
  };

  const goToFirstPage = () => goToPage(1);
  const goToPreviousPage = () => goToPage(currentPage - 1);
  const goToNextPage = () => goToPage(currentPage + 1);
  const goToLastPage = () => goToPage(totalPages);

  if (loading) {
    return (
      <div className="flex justify-center items-center h-40">
        <p className="text-muted-foreground">Loading telemetry data...</p>
      </div>
    );
  }

  if (!telemetry.length) {
    return (
      <div className="flex justify-center items-center h-40 border rounded-lg">
        <p className="text-muted-foreground">No telemetry data available.</p>
      </div>
    );
  }

  return (
    <div>
      <div className="flex items-center justify-between mb-4">
        <h3 className="text-lg font-medium">Telemetry Data</h3>
        <Select
          value={recordsPerPage}
          onValueChange={(value) => {
            setRecordsPerPage(value);
            setCurrentPage(1); // Reset to first page when changing records per page
          }}
        >
          <SelectTrigger className="w-[180px]">
            <SelectValue placeholder="Records per page" />
          </SelectTrigger>
          <SelectContent>
            <SelectItem value="5">5 records per page</SelectItem>
            <SelectItem value="10">10 records per page</SelectItem>
            <SelectItem value="25">25 records per page</SelectItem>
            <SelectItem value="50">50 records per page</SelectItem>
          </SelectContent>
        </Select>
      </div>

      <div className="rounded-md border">
        <div className="overflow-x-auto">
          <table className="w-full">
            <thead>
              <tr className="bg-secondary/50">
                <th className="text-left p-2 font-medium">Time</th>
                <th className="text-left p-2 font-medium">Battery</th>
                <th className="text-left p-2 font-medium">Temp</th>
                <th className="text-left p-2 font-medium">Location</th>
              </tr>
            </thead>
            <tbody>
              {currentRecords.map((record) => (
                <tr key={record.id} className="border-t">
                  <td className="p-2">
                    <div>
                      {new Date(record.timestamp).toLocaleString()}
                    </div>
                    <div className="text-xs text-muted-foreground">
                      {formatDistance(new Date(record.timestamp), new Date(), {
                        addSuffix: true,
                      })}
                    </div>
                  </td>
                  <td className="p-2">{record.battery_level}%</td>
                  <td className="p-2">{record.temperature}°C</td>
                  <td className="p-2">
                    {record.location
                      ? `${record.location.latitude.toFixed(
                          4
                        )}, ${record.location.longitude.toFixed(4)}`
                      : "N/A"}
                  </td>
                </tr>
              ))}
            </tbody>
          </table>
        </div>
      </div>

      <div className="flex justify-between items-center mt-4">
        <div className="text-sm text-muted-foreground">
          Showing {startIndex + 1}-{Math.min(endIndex, telemetry.length)} of{" "}
          {telemetry.length} records
        </div>
        <div className="flex items-center space-x-2">
          <Button
            variant="outline"
            size="sm"
            onClick={goToFirstPage}
            disabled={currentPage === 1}
          >
            <ChevronsLeft className="h-4 w-4" />
          </Button>
          <Button
            variant="outline"
            size="sm"
            onClick={goToPreviousPage}
            disabled={currentPage === 1}
          >
            <ChevronLeft className="h-4 w-4" />
          </Button>
          <span className="text-sm">
            Page {currentPage} of {totalPages || 1}
          </span>
          <Button
            variant="outline"
            size="sm"
            onClick={goToNextPage}
            disabled={currentPage >= totalPages}
          >
            <ChevronRight className="h-4 w-4" />
          </Button>
          <Button
            variant="outline"
            size="sm"
            onClick={goToLastPage}
            disabled={currentPage >= totalPages}
          >
            <ChevronsRight className="h-4 w-4" />
          </Button>
        </div>
      </div>
    </div>
  );
}
