import { useCallback, useState } from "react";
import "./App.scss";
import CodeEditor from "@uiw/react-textarea-code-editor";
import { kppApi } from "./api/apiClient";
import { DataList } from "./DataList/DataList";
import { TablesModal } from "./TablesView/TablesModal";
import { Aside } from "./ui/Aside";
import useAsideStore from "./store/AsideStore";
import { shallow } from "zustand/shallow";
import { BeatLoader } from "react-spinners";

const apiClient = new kppApi();

function App() {
  const [showModal, setShowModal] = useState<boolean>(false);
  const [loading, setLoading] = useState<boolean>(false);
  const { setCode, setNodes, nullNodes } = useAsideStore((state) => state, shallow);
  const nodes = useAsideStore((state) => state.nodes, shallow);
  const code = useAsideStore((state) => state.code, shallow);

  const parser = useCallback(
    (data: { data: object }) => {
      Object.entries(data.data).map((item, index) => {
        if (item[0] && item[1]) {
          setNodes(
            <DataList key={index} key1={index} title={item[0]} list={item[1]} />
          );
        }
      });
    },
    [setNodes]
  );

  const sendData = useCallback(() => {
    nullNodes();
    setLoading(true);
    apiClient
      .post(code)
      .then((res) => parser(res.data))
      .then(() => { setShowModal(true); setLoading(false) })
  }, [code, setNodes]);

  return (
    <>
      <TablesModal
        nodes={nodes}
        show={showModal}
        onExit={() => setShowModal(false)}
      />
      <section className="main-wrapper">
        <Aside />
        <CodeEditor
          value={code}
          language="json"
          placeholder="Вставьте код JSON."
          className="code-editor"
          onChange={(evn) => setCode(evn.target.value)}
          padding={15}
          style={{
            fontSize: 12,
            backgroundColor: "#f5f5f5",
            fontFamily:
              "ui-monospace,SFMono-Regular,SF Mono,Consolas,Liberation Mono,Menlo,monospace",
          }}
        />
        {code &&
          <button className="send-btn" onClick={() => sendData()}>
            Запустить<BeatLoader color="#fff" loading={loading} size={4}/>
          </button>
        }
      </section>
    </>
  );
}

export default App;
