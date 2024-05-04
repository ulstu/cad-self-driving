import { ReactElement } from "react";
import { create } from "zustand";
import { devtools } from "zustand/middleware";

interface IAsideStore {
  code?: string;
  nodes?: ReactElement[];
  setNodes: (node: ReactElement) => void;
  setCode: (code: string) => void;
  nullNodes: () => void;
}

const useAsideStore = create<IAsideStore>()(
  devtools(
    (set, get) => ({
      code: undefined,
      nodes: [],
      setCode: (code: string) => {
        set({ code: code }, false);
      },
      nullNodes: () => {
        set({ nodes: undefined }, false);
      },
      setNodes: (node: ReactElement) => {
        set({ nodes: [...get().nodes || [], node] }, false);
      },
    }),
    {
      name: "riasHcsStore",
    }
  )
);

export default useAsideStore;
