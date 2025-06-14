import { type Configuration, type ErrorMessage } from "./APIObjects";


let apiUrl: string = "https://31898643-ecc4-4cd8-9932-0f2848cf0fca.mock.pstmn.io"

export async function getBots(): Promise<Configuration | Error> {
    try {
        console.log("eat a dick")
        const response = await fetch(`${apiUrl}/bots`);
        if (!response.ok) {
            throw new Error(`HTTP error! status: ${response.status}`);
        }
        const data = await response.json() as Configuration;
        return data;
    } catch (error) {
        console.error("Error fetching bots:", error);
        return Promise.reject(Error(`HTTP error! status: ${error}`))
    }
}