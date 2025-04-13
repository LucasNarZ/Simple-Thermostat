import axios from "axios"

export function changeSetPoint(newValue:number){
    const endpoint = import.meta.env.VITE_API_ENDPOINT
    axios.post(endpoint + "/shadow", {
        deviceName:"testEsp",
        shadow:{
            state:{
                desired:{
                  setPoint: newValue
                }
              }
        }
    })
}

export function changeStatus(newStatus:string){
    const endpoint = import.meta.env.VITE_API_ENDPOINT
    axios.post(endpoint + "/shadow", {
        deviceName:"testEsp",
        shadow:{
            state:{
                desired:{
                  status: newStatus
                }
              }
        }
    })
}