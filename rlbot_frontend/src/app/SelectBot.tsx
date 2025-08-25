export default function SelectBot({opts, setBot}: 
    {
        opts: string[], 
        setBot: (bot: string) => void
    }) {

        function selectBot(e: React.ChangeEvent<HTMLSelectElement>) {
            const selectedBot = e.target.value;
            setBot(selectedBot);
        }

        return (
            <div>
                <p>List of Bots</p>
                <select
                name="bots"
                id="bots_select"
                onChange={selectBot}
                className="bot-select"
                >
                {opts && opts.length > 0 &&
                    opts.map((bot, index) => {
                    return (
                        <option
                        key={bot + index}
                        value={bot}
                        style={{ marginBottom: "20px" }}
                        >
                        {bot}
                        </option>
                    );
                    })}
                </select>
            </div>
        );
}