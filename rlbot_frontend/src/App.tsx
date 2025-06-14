import { useState } from 'react'
import { useEffect } from 'react'
import { getBots } from './ApiUtilities'
import type { Configuration, ErrorMessage } from './APIObjects'
import reactLogo from './assets/react.svg'
import viteLogo from '/vite.svg'
import './App.css'

export default function App() {
  const [configs, setConfig] = useState<Configuration>({} as Configuration);
  const [bot, setBot] = useState("");
  const [pipeline, setPipeline] = useState<Array<string>>([]);
  const [availableComponents, setAvailableComponents] = useState<Array<string>>([]);
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

        if(response['bots'].length)
          setBot(response["bots"][0]); // Set the first bot as the default
        if(response['components'].length)
          setAvailableComponents(response["components"]); // Set available components
      }
    }

    fetchBots();
  }, []); // Empty dependency array means this effect runs only once

  function selectBot(e) {
    const selectedBot = e.target.value;
    setBot(selectedBot);
  }

  function dragging(e) {
    if(draggingComponent != ""){
      setDraggingComponent("");
      setAvailableComponents(
        configs["components"].filter((component) => {
          return !pipeline.find((used) => used == component)
        })
      )
    }

    const component = e.target.getAttribute("data-component")
    
    setDraggingComponent(component)
    setAvailableComponents(
      configs["components"].filter((component) => {
        return !pipeline.find((used) => used == component) && component != draggingComponent
      })
    )
  }

  function dragOver(e) {
    e.preventDefault();
    e.stopPropagation();
    if (draggingComponent !== "") {
      e.dataTransfer.dropEffect = "none"; // Indicate that this is a move operation
    } else {
      e.dataTransfer.dropEffect = "move"; // Indicate that this is a copy operation
      pipeline.push(draggingComponent);
      setPipeline(pipeline);
    }
  }

  function drop(e) {
    setDraggingComponent("")
    setAvailableComponents(
      configs["components"].filter((component) => {
        return !pipeline.find((used) => used == component)
      })
    )
  }
  
  return (
    <>
      <div>
        <a href="https://vite.dev" target="_blank">
          <img src={viteLogo} className="logo" alt="Vite logo" />
        </a>
        <a href="https://react.dev" target="_blank">
          <img src={reactLogo} className="logo react" alt="React logo" />
        </a>
      </div>

      <h1>RLBot Control Panel</h1>
      <div>
        <p>List of Bots</p>
        <select 
          name="bots"
          id="bots_select"
          onChange={selectBot} 
          style={{ width: '100%', height: '200px', marginBottom: '20px' }}
        >
          {
            Object.keys(configs).length > 0 && configs["bots"].length > 0 &&
                configs["bots"].map((bot, index) => {
                  return (
                    <option 
                      key={bot + index}
                      value={bot}
                      style={{ marginBottom: '20px'}}
                    >
                      {bot}
                    </option>
                  )
                })
          }
        </select>
      </div>

      <div className="card">
        <h2> Available Components </h2>
        <div 
          className="availableComponents" 
          onDragLeave={dragging}
          style={{ display: "flex", "flexDirection": "row", "flexWrap": "wrap", "gap": "20px", "marginBottom": "20px" }}>
          {
              availableComponents.length > 0 &&
                availableComponents.map((comp, index) => {
                  return (
                    <div 
                      data-component={comp}
                      draggable="true"
                      onDrop={drop}
                      key={comp + index}
                      style={{ marginBottom: '20px'}}
                    >
                      {comp}
                    </div>
                  )
                })
          }
        </div>
      </div>

      <div className="card">
        <h2> Set Pipeline </h2>
        <div 
          className="availableComponents" 
          style={{ display: "flex", "flexDirection": "row", "flexWrap": "wrap", "gap": "20px", "marginBottom": "20px" }}
          onDragOver={dragOver}
          onDrop={drop}
        >
        {
            pipeline.length > 0 &&
              pipeline.map((comp, index) => {
                return (
                  <div 
                    data-component={comp}
                    draggable="true"
                    onDrop={drop}
                    key={comp + index}
                    style={{ marginBottom: '20px'}}
                  >
                    {bot}
                  </div>
                )
              })
        }
        </div>
      </div>

      <p className="read-the-docs">
        Click on the Vite and React logos to learn more
      </p>
    </>
  )
}
