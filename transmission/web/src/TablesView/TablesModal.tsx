import { ReactElement } from "react";
import "./TablesModal.scss";
import { IoClose } from "react-icons/io5";

interface Props {
  show: boolean;
  nodes?: ReactElement[];
  onExit: () => void;
}

export const TablesModal = (props: Props) => {
  return (
    <section
      className="tables-modal"
      style={{ top: props.show ? 10 : window.innerHeight }}
    >
      <div className="header">
        <button onClick={props.onExit}>
          <IoClose size={32} />
        </button>
      </div>
      <section className="tables-section">
        {props.nodes && props.nodes.map((item) => item)}
      </section>
    </section>
  );
};
