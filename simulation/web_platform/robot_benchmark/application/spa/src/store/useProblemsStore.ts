import { create } from "zustand";

interface ProblemsStore {
    vs_code?: number;
    webots_stream_port?: number;
    problem?: number;
    robot_panel_port?: number;
    setData: (vs: number, webots: number, problem: number, robot_panel: number) => void; 
}

export const useProblemsStore = create<ProblemsStore>((set) => ({
    setData: (vs: number, webots: number, problem: number, robot_panel: number) => set({vs_code: vs, webots_stream_port: webots, problem: problem, robot_panel_port: robot_panel})
  }));