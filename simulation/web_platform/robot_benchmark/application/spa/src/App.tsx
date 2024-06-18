import { useEffect } from "react";
import AppLayout from "./AppLayout";
import { Login } from "./components/Auth/Login/Login";
import { useAuthStore } from "./store/useAuthStore";

export const App = () => {
  const { isAuthenticated } = useAuthStore();

  useEffect(() => {
    console.log(isAuthenticated);
  }, [isAuthenticated]);

  return isAuthenticated ? <AppLayout /> : <Login />;
};
