
import { useState } from "react";
import { TelemetryRecord } from "@/hooks/useTelemetryHistory";
import {
  Table,
  TableBody,
  TableCell,
  TableHead,
  TableHeader,
  TableRow,
} from "@/components/ui/table";
import { Input } from "@/components/ui/input";
import { Badge } from "@/components/ui/badge";
import {
  Pagination,
  PaginationContent,
  PaginationItem,
  PaginationLink,
  PaginationNext,
  PaginationPrevious,
} from "@/components/ui/pagination";

interface TelemetryTableProps {
  telemetry: TelemetryRecord[];
}

export default function TelemetryTable({ telemetry }: TelemetryTableProps) {
  const [currentPage, setCurrentPage] = useState(1);
  const [searchQuery, setSearchQuery] = useState("");
  const recordsPerPage = 10;

  const filteredData = telemetry.filter((record) =>
    record.formattedDate?.toLowerCase().includes(searchQuery.toLowerCase()) ||
    record.status?.toLowerCase().includes(searchQuery.toLowerCase())
  );

  const indexOfLastRecord = currentPage * recordsPerPage;
  const indexOfFirstRecord = indexOfLastRecord - recordsPerPage;
  const currentRecords = filteredData.slice(indexOfFirstRecord, indexOfLastRecord);
  const totalPages = Math.ceil(filteredData.length / recordsPerPage);

  function renderStatusBadge(status: string) {
    if (status === "OK") return <Badge variant="outline" className="bg-green-50 text-green-700">OK</Badge>;
    if (status === "WARNING") return <Badge variant="secondary">Warning</Badge>;
    if (status === "ERROR") return <Badge variant="destructive">Error</Badge>;
    return <Badge variant="outline">{status}</Badge>;
  }

  return (
    <div className="space-y-4">
      <div>
        <Input
          placeholder="Search by date or status..."
          value={searchQuery}
          onChange={(e) => setSearchQuery(e.target.value)}
          className="max-w-sm"
        />
      </div>

      <div className="rounded-md border">
        <Table>
          <TableHeader>
            <TableRow>
              <TableHead>Date & Time</TableHead>
              <TableHead>Battery Level</TableHead>
              <TableHead>Temperature</TableHead>
              <TableHead>Status</TableHead>
              <TableHead>Location</TableHead>
            </TableRow>
          </TableHeader>
          <TableBody>
            {currentRecords.map((record) => (
              <TableRow key={record.id}>
                <TableCell>
                  <div className="font-medium">{record.formattedDate}</div>
                  <div className="text-sm text-muted-foreground">{record.formattedTime}</div>
                </TableCell>
                <TableCell>{record.batteryLevel !== null ? `${record.batteryLevel}%` : "N/A"}</TableCell>
                <TableCell>{record.temperature !== null ? `${record.temperature}°C` : "N/A"}</TableCell>
                <TableCell>{renderStatusBadge(record.status)}</TableCell>
                <TableCell>
                  {record.location ? (
                    <span className="text-xs">
                      {record.location.latitude.toFixed(5)}, {record.location.longitude.toFixed(5)}
                    </span>
                  ) : (
                    "N/A"
                  )}
                </TableCell>
              </TableRow>
            ))}
            {currentRecords.length === 0 && (
              <TableRow>
                <TableCell colSpan={5} className="h-24 text-center">
                  No telemetry records found
                </TableCell>
              </TableRow>
            )}
          </TableBody>
        </Table>
      </div>
      
      {totalPages > 1 && (
        <Pagination>
          <PaginationContent>
            <PaginationItem>
              <PaginationPrevious
                onClick={() => setCurrentPage(p => Math.max(1, p - 1))}
                disabled={currentPage === 1}
              />
            </PaginationItem>
            
            {Array.from({ length: Math.min(5, totalPages) }).map((_, i) => {
              const pageNum = i + 1;
              return (
                <PaginationItem key={i}>
                  <PaginationLink
                    onClick={() => setCurrentPage(pageNum)}
                    isActive={currentPage === pageNum}
                  >
                    {pageNum}
                  </PaginationLink>
                </PaginationItem>
              );
            })}
            
            {totalPages > 5 && currentPage < totalPages && (
              <>
                <PaginationItem>
                  <PaginationLink disabled>...</PaginationLink>
                </PaginationItem>
                <PaginationItem>
                  <PaginationLink onClick={() => setCurrentPage(totalPages)}>
                    {totalPages}
                  </PaginationLink>
                </PaginationItem>
              </>
            )}
            
            <PaginationItem>
              <PaginationNext
                onClick={() => setCurrentPage(p => Math.min(totalPages, p + 1))}
                disabled={currentPage === totalPages}
              />
            </PaginationItem>
          </PaginationContent>
        </Pagination>
      )}
    </div>
  );
}
