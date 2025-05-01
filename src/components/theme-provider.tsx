
"use client"

import * as React from "react"
import { ThemeProvider as NextThemesProvider } from "next-themes"
import { type ThemeProviderProps } from "next-themes/dist/types"

export function ThemeProvider({ children, ...props }: ThemeProviderProps) {
  // Make sure to provide the attribute prop for applying the theme class to the HTML element
  return <NextThemesProvider attribute="class" {...props}>{children}</NextThemesProvider>
}
