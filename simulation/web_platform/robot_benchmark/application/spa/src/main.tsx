import React from "react";
import ReactDOM from "react-dom/client";
import "./index.css";
import { createBrowserRouter, RouterProvider } from "react-router-dom";
import { TournamentList } from "./components/Tournament/TournamentList.tsx";
import { App } from "./App.tsx";
import { LeaderboardList } from "./components/Leaderboard/LeaderboardList.tsx";
import ProblemsList from "./components/Problems/ProblemsList.tsx";
import ProblemDetail from "./components/Problems/ProblemDetail.tsx";
import { TournamentDetail } from "./components/Tournament/TournamentDetail.tsx";
import { Register } from "./components/Auth/Register/Register.tsx";
import { UsersList } from "./components/Users/UsersList.tsx";

const router = createBrowserRouter([
  {
    path: "/",
    element: <App />,
    children: [
      {
        index: true,
        element: <LeaderboardList />,
      },
      {
        path: "/tournaments",
        element: <TournamentList />,
      },
      {
        path: "/tournaments/:id",
        element: <TournamentDetail />,
      },
      {
        path: "/leaderboard",
        element: <LeaderboardList />,
      },
      {
        path: "/problems",
        element: <ProblemsList />,
      },
      {
        path: "/problems/:id",
        element: <ProblemDetail />,
      },
      {
        path: "/users",
        element: <UsersList />
      }
    ],
  },
  {
    path: "/register",
    element: <Register />,
  },
]);

ReactDOM.createRoot(document.getElementById("root")!).render(
  <React.StrictMode>
    <RouterProvider router={router} />
  </React.StrictMode>
);
