
import { create } from 'zustand';
import { devtools, persist } from 'zustand/middleware';
import { Robot } from '@/types/robot';
import { supabase } from '@/integrations/supabase/client';

// Define the store state interface
interface RobotState {
  // Robot data
  robots: Robot[];
  selectedRobotId: string | null;
  filteredRobots: Robot[];
  isLoading: boolean;
  error: Error | null;
  
  // Filter state
  filters: {
    status: string[];
    types: string[];
    search: string;
  };
  
  // Telemetry retention
  retentionDays: number;
  
  // Actions
  fetchRobots: () => Promise<void>;
  selectRobot: (id: string | null) => void;
  addRobot: (robot: Omit<Robot, 'id'>) => Promise<Robot | null>;
  updateRobot: (id: string, updates: Partial<Robot>) => Promise<void>;
  deleteRobot: (id: string) => Promise<boolean>;
  setFilter: (filterType: string, values: any) => void;
  clearFilters: () => void;
  setRetentionDays: (days: number) => Promise<void>;
  updateRobotFromTelemetry: (robotId: string, telemetryData: any) => void;
}

// Create the store
export const useRobotStore = create<RobotState>()(
  devtools(
    persist(
      (set, get) => ({
        // Initial state
        robots: [],
        selectedRobotId: null,
        filteredRobots: [],
        isLoading: false,
        error: null,
        filters: {
          status: [],
          types: [],
          search: '',
        },
        retentionDays: 7,
        
        // Fetch robots from Supabase
        fetchRobots: async () => {
          set({ isLoading: true, error: null });
          try {
            const { data: sessionData } = await supabase.auth.getSession();
            if (!sessionData.session) {
              throw new Error('Not authenticated');
            }
            
            const { data, error } = await supabase
              .from('robots')
              .select('*')
              .eq('user_id', sessionData.session.user.id);
              
            if (error) throw error;
            
            // Map the data to our Robot type
            const mappedRobots = data.map(robot => ({
              id: robot.id,
              name: robot.name,
              model: robot.type,
              status: robot.status || 'offline',
              lastHeartbeat: robot.last_ping || new Date().toISOString(),
              batteryLevel: robot.battery_level || 0,
              temperature: robot.temperature || 0,
              location: robot.location,
              ipAddress: '',
              errorCount: 0,
              apiKey: robot.api_key,
              telemetryData: robot.telemetry_data,
            }));
            
            set({ 
              robots: mappedRobots, 
              filteredRobots: mappedRobots,
              isLoading: false,
            });
          } catch (error) {
            console.error('Error fetching robots:', error);
            set({ error: error as Error, isLoading: false });
          }
        },
        
        // Select a robot by ID
        selectRobot: (id) => {
          set({ selectedRobotId: id });
        },
        
        // Add a new robot
        addRobot: async (robot) => {
          try {
            const { data: sessionData } = await supabase.auth.getSession();
            if (!sessionData.session) return null;
            
            const { data, error } = await supabase
              .from('robots')
              .insert([{
                name: robot.name,
                type: robot.model,
                description: '',
                user_id: sessionData.session.user.id,
                api_key: crypto.randomUUID().replace(/-/g, '')
              }])
              .select();
              
            if (error) throw error;
            
            if (data && data.length > 0) {
              const newRobot: Robot = {
                id: data[0].id,
                name: data[0].name,
                model: data[0].type,
                status: 'offline',
                lastHeartbeat: data[0].last_ping || new Date().toISOString(),
                batteryLevel: data[0].battery_level || 0,
                temperature: data[0].temperature || 0,
                location: data[0].location,
                ipAddress: '',
                errorCount: 0,
                apiKey: data[0].api_key,
              };
              
              set(state => ({
                robots: [...state.robots, newRobot],
                filteredRobots: [...state.robots, newRobot],
              }));
              
              return newRobot;
            }
            return null;
          } catch (error) {
            console.error('Error adding robot:', error);
            set({ error: error as Error });
            return null;
          }
        },
        
        // Update robot details
        updateRobot: async (id, updates) => {
          try {
            // First update the local state optimistically
            set(state => ({
              robots: state.robots.map(robot => 
                robot.id === id ? { ...robot, ...updates } : robot
              ),
              filteredRobots: state.filteredRobots.map(robot => 
                robot.id === id ? { ...robot, ...updates } : robot
              )
            }));
            
            // Then update in Supabase
            // Map our Robot type to Supabase robot type
            const supabaseUpdates: any = {};
            if (updates.name) supabaseUpdates.name = updates.name;
            if (updates.model) supabaseUpdates.type = updates.model;
            if (updates.status) supabaseUpdates.status = updates.status;
            if (updates.batteryLevel !== undefined) supabaseUpdates.battery_level = updates.batteryLevel;
            if (updates.temperature !== undefined) supabaseUpdates.temperature = updates.temperature;
            if (updates.location) supabaseUpdates.location = updates.location;
            if (updates.telemetryData) supabaseUpdates.telemetry_data = updates.telemetryData;
            
            const { error } = await supabase
              .from('robots')
              .update(supabaseUpdates)
              .eq('id', id);
              
            if (error) throw error;
          } catch (error) {
            console.error('Error updating robot:', error);
            set({ error: error as Error });
            
            // Revert the optimistic update if there was an error
            get().fetchRobots(); 
          }
        },
        
        // Delete a robot
        deleteRobot: async (id) => {
          try {
            const { error } = await supabase
              .from('robots')
              .delete()
              .eq('id', id);
              
            if (error) throw error;
            
            set(state => ({
              robots: state.robots.filter(robot => robot.id !== id),
              filteredRobots: state.filteredRobots.filter(robot => robot.id !== id),
              selectedRobotId: state.selectedRobotId === id ? null : state.selectedRobotId
            }));
            
            return true;
          } catch (error) {
            console.error('Error deleting robot:', error);
            set({ error: error as Error });
            return false;
          }
        },
        
        // Set filter values
        setFilter: (filterType, values) => {
          set(state => {
            // Update the filter state
            const newFilters = {
              ...state.filters,
              [filterType]: values
            };
            
            // Apply filters to robots
            let filtered = [...state.robots];
            
            // Filter by status
            if (newFilters.status && newFilters.status.length > 0) {
              filtered = filtered.filter(robot => newFilters.status.includes(robot.status));
            }
            
            // Filter by type/model
            if (newFilters.types && newFilters.types.length > 0) {
              filtered = filtered.filter(robot => newFilters.types.includes(robot.model));
            }
            
            // Filter by search term
            if (newFilters.search) {
              const searchTerm = newFilters.search.toLowerCase();
              filtered = filtered.filter(robot => 
                robot.name.toLowerCase().includes(searchTerm) || 
                robot.model.toLowerCase().includes(searchTerm)
              );
            }
            
            return {
              filters: newFilters,
              filteredRobots: filtered,
            };
          });
        },
        
        // Clear all filters
        clearFilters: () => {
          set(state => ({
            filters: {
              status: [],
              types: [],
              search: '',
            },
            filteredRobots: state.robots,
          }));
        },
        
        // Set telemetry retention days
        setRetentionDays: async (days) => {
          try {
            const { data: sessionData } = await supabase.auth.getSession();
            if (!sessionData.session) return;
            
            const { error } = await supabase
              .from('profiles')
              .update({ telemetry_retention_days: days })
              .eq('id', sessionData.session.user.id);
              
            if (error) throw error;
            
            set({ retentionDays: days });
          } catch (error) {
            console.error('Error updating retention days:', error);
          }
        },
        
        // Update robot state from telemetry data
        updateRobotFromTelemetry: (robotId, telemetryData) => {
          set(state => {
            const updatedRobots = state.robots.map(robot => {
              if (robot.id === robotId) {
                const updates: Partial<Robot> = {
                  lastHeartbeat: new Date().toISOString(),
                  status: 'online',
                };
                
                if (telemetryData.batteryLevel !== undefined) {
                  updates.batteryLevel = telemetryData.batteryLevel;
                }
                
                if (telemetryData.temperature !== undefined) {
                  updates.temperature = telemetryData.temperature;
                }
                
                if (telemetryData.location) {
                  updates.location = {
                    latitude: telemetryData.location.latitude || telemetryData.location.lat || 0,
                    longitude: telemetryData.location.longitude || telemetryData.location.lng || 0,
                  };
                }
                
                if (telemetryData.customTelemetry) {
                  updates.telemetryData = {
                    ...(robot.telemetryData || {}),
                    ...telemetryData.customTelemetry
                  };
                }
                
                return { ...robot, ...updates };
              }
              return robot;
            });
            
            const updatedFilteredRobots = state.filteredRobots.map(robot => {
              if (robot.id === robotId) {
                const matchingRobot = updatedRobots.find(r => r.id === robotId);
                return matchingRobot || robot;
              }
              return robot;
            });
            
            return {
              robots: updatedRobots,
              filteredRobots: updatedFilteredRobots
            };
          });
        }
      }),
      {
        name: 'robot-storage',
        partialize: (state) => ({ 
          retentionDays: state.retentionDays,
          selectedRobotId: state.selectedRobotId,
          filters: state.filters
        }),
      }
    )
  )
);
