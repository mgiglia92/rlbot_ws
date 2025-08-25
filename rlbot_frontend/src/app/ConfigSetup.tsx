import { useState, useEffect } from "react";
import { getBots } from "../utilities/ApiUtilities";
import type { Configuration } from "../utilities/ApiObjects";
import { NavLink } from "react-router-dom";

import SelectBot from "./SelectBot.tsx";

import reactLogo from "../assets/react.svg";
import rlbot_logo from "../assets/rlbot_logo.png";
import "./App.css";

export default function ConfigSetup() {
  const [configs, setConfig] = useState<Configuration>({models: [], components: []} as Configuration);
  const [bot, setBot] = useState("");
  const [availableComponents, setAvailableComponents] = useState<string[]>([]);
  const [pipeline, setPipeline] = useState<string[]>([]);

  const [draggingComponent, setDraggingComponent] = useState("");

  useEffect(() => { 
    async function fetchBots() {
      const response = await getBots();

      if (response instanceof Error) {
        console.error("Error fetching bots:", response);
        return;
      }

      if (response && Object.keys(response).length > 0) {
        setConfig(response);

        if (response["models"].length) setBot(response["models"][0]); // Set the first bot as the default
        if (response["components"].length) setAvailableComponents(response["components"]); // Set available components
      }
    }

    fetchBots();
  }, []); // Empty dependency array means this effect runs only once

  function dragBegin(e: React.DragEvent<HTMLDivElement>) {
    const target = e.target as HTMLDivElement;

    if (draggingComponent != "") {
      resetDrag(e);
    }

    const component = target.getAttribute("data-component") ?? "";

    console.log("starting drag with component:", component);
    
    setDraggingComponent(component);
    setAvailableComponents(
      configs["components"].filter(component => {
        return (
          !pipeline.find(used => used == component) &&
          component != draggingComponent
        );
      }),
    );
    setPipeline(
      pipeline.filter(used => used != draggingComponent),
    );
  }

  function drop(e: React.DragEvent<HTMLDivElement>) {
    e.preventDefault();
    e.stopPropagation();
    const target = e.target as HTMLDivElement;
    const dropTarget = target.getAttribute("data-target") ?? "";
    
    if (draggingComponent == "") {
      console.log("no component being dragged");
      return;
    }

    console.log("dropped component:", draggingComponent, dropTarget);

    if (dropTarget == "pipeline" && !pipeline.includes(draggingComponent)) {
      setPipeline(
        pipeline.concat([draggingComponent]),
      )
    } else if (dropTarget == "availableComponents" && !availableComponents.includes(draggingComponent)) {
      setAvailableComponents(
        availableComponents.concat([draggingComponent])
      )
    }
    resetDrag(e);;
  }

  function resetDrag(e: React.DragEvent<HTMLDivElement>) {
    e.preventDefault();

    console.log("resetting drag");
    setAvailableComponents(
      configs["components"].filter(component => {
        return !pipeline.find(used => used == component);
      }),
    );
    setDraggingComponent("");
  }

  function dragHover(e: React.DragEvent<HTMLDivElement>) {
    e.preventDefault();
  }

  function dragLeave(e: React.DragEvent<HTMLDivElement>) {
    e.preventDefault();
    e.stopPropagation();
    const leaveTarget = e.target as HTMLDivElement
    if(draggingComponent == "") 
      return;

    if(leaveTarget.getAttribute("data-target") == "pipeline") {
      setPipeline(
        pipeline.filter(component => component != draggingComponent)
      );
    }

    if(leaveTarget.getAttribute("data-target") == "availableComponents") {
      console.log("availcomps on leave:", availableComponents.filter(component => component != draggingComponent));
      setAvailableComponents(
        availableComponents.filter(component => component != draggingComponent)
      );
    }
  }

  function dragEnter(e: React.DragEvent<HTMLDivElement>) {
    e.preventDefault();
    e.stopPropagation();
    const enterTarget = e.target as HTMLDivElement

    if(draggingComponent == "") {
      return;
    }

    if(enterTarget.getAttribute("data-target") == "pipeline" && !pipeline.includes(draggingComponent)) {
      console.log("pipeline on enter:", pipeline.concat([draggingComponent]));
      setPipeline(
        pipeline.concat([draggingComponent])
      )
    }

    if(enterTarget.getAttribute("data-target") == "availableComponents" && !availableComponents.includes(draggingComponent)) {
      console.log("availcomp on enter:", availableComponents.concat([draggingComponent]));
      setAvailableComponents(
        availableComponents.concat([draggingComponent])
      )
    }
  }

  return (
    <>
      <div>
        <a href="https://rlbot.org/" target="_blank">
          <img src={rlbot_logo} className="logo" alt="Vite logo" />
        </a>
        <a href="https://react.dev" target="_blank">
          <img src={reactLogo} className="logo react" alt="React logo" />
        </a>
      </div>

      <h1>RLBot Control Panel</h1>
      <SelectBot opts={configs["models"]} setBot={setBot}></SelectBot>


      <div className="card">
        <h2> Available Components </h2>
        <div
          className="drag-drop-target"
          data-target="availableComponents"
          onDragLeave={dragLeave}
          onDragEnter={dragEnter}
          onDragOver={dragHover}
          onDrop={drop}
        >
          {availableComponents.length > 0 &&
            availableComponents.map((comp, index) => {
              return (
                <div
                  data-component={comp}
                  draggable="true"
                  onDragStart={dragBegin}
                  onDragEnd={resetDrag}
                  key={comp + index}
                  style={{ marginBottom: "20px" }}
                >
                  {comp}
                </div>
              );
            })}
        </div>
      </div>

      <div className="card">
        <h2> Set Pipeline </h2>
        <div
          className="drag-drop-target"
          data-target="pipeline"
          onDragLeave={dragLeave}
          onDragEnter={dragEnter}
          onDragOver={dragHover}
          onDrop={drop}
        >
          {pipeline.length > 0 &&
            pipeline.map((comp, index) => {
              return (
                <div
                  data-component={comp}
                  draggable="true"
                  onDragStart={dragBegin}
                  onDragEnd={resetDrag}
                  key={comp + index}
                  style={{ marginBottom: "20px" }}
                >
                  {comp}
                </div>
              );
            })}
        </div>
      </div>

    <NavLink to="/plotter">
        <button
            onClick={() => {
            console.log("Pipeline:", pipeline);
            console.log("Selected Bot:", bot);
            // Here you would typically send the pipeline and bot to the backend
            }}
        >
            Start
        </button>
    </NavLink>

      <p className="read-the-docs">
        Click on the Vite and React logos to learn more
      </p>
    </>
  );
}
