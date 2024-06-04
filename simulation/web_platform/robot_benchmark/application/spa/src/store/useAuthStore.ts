import { create } from "zustand";

interface AuthState {
  isAuthenticated: boolean;
  token: string | undefined;
  userId?: number;
  is_super?: boolean;
  login: (token: string, userId: number, is_super: boolean) => void;
  logout: () => void;
}

export const useAuthStore = create<AuthState>((set) => ({
  isAuthenticated: false,
  token: undefined,
  login: (token: string, userId: number, is_super: boolean) => {console.log(token, userId);set({ isAuthenticated: true, token, userId, is_super: true })},
  logout: () => set({ isAuthenticated: false, token: undefined }),
}));
