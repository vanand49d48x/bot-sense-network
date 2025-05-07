
import React from "react";

interface CodeBlockProps {
  code: string;
  language?: string;
  className?: string;
}

export function CodeBlock({ code, language = "javascript", className = "" }: CodeBlockProps) {
  return (
    <div className={`overflow-x-auto ${className}`}>
      <pre className="p-4 bg-background border rounded-md text-sm">
        <code className="language-javascript">{code}</code>
      </pre>
    </div>
  );
}

// Add Code component as an alias for CodeBlock for backward compatibility
export const Code = CodeBlock;
