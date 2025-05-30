
import { defineConfig } from "vite";
import react from "@vitejs/plugin-react-swc";
import path from "path";
import { componentTagger } from "lovable-tagger";

// https://vitejs.dev/config/
export default defineConfig(({ mode }) => ({
  server: {
    host: "::",
    port: 8080,
  },
  plugins: [
    react(),
    mode === 'development' &&
    componentTagger(),
  ].filter(Boolean),
  resolve: {
    alias: {
      "@": path.resolve(__dirname, "./src"),
    },
    dedupe: ['react-leaflet', 'leaflet']
  },
  build: {
    commonjsOptions: {
      transformMixedEsModules: true, // Enable this option to handle mixed ES modules
    },
    rollupOptions: {
      external: [],
      output: {
        manualChunks: {
          leaflet: ['leaflet', 'react-leaflet'],
        }
      }
    }
  }
}));
