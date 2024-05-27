import AppLayout from "./AppLayout";
import { Login } from "./Login";
import { useAuthStore } from "./store/useAuthStore";

export const App = () => {
  const { isAuthenticated } = useAuthStore();

  return isAuthenticated ? <AppLayout /> : <Login />;
};
