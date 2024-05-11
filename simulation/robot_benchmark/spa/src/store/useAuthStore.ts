import { create } from "zustand";

interface AuthState {
  isAuthenticated: boolean;
  token: string | undefined;
  login: (token: string) => void;
  logout: () => void;
}

export const useAuthStore = create<AuthState>((set) => ({
  isAuthenticated: false,
  token: undefined,
  login: (token: string) => set({ isAuthenticated: true, token }),
  logout: () => set({ isAuthenticated: false, token: undefined }),
}));
