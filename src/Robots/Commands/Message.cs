namespace Robots.Commands;

public class Message : Command
{
    readonly string _message;

    public Message(string message)
    {
        _message = message;
    }

    protected override void Populate()
    {
        _commands.Add(Manufacturers.ABB, (_, _) => $"TPWrite \"{_message}\";");
        _commands.Add(Manufacturers.KUKA, (_, _) => $"; \"{_message}\"");
        _commands.Add(Manufacturers.UR, (_, _) => $"textmsg(\"{_message}\")");
        _commands.Add(Manufacturers.Staubli, (_, _) => $"putln(\"{_message}\")");
        _commands.Add(Manufacturers.FrankaEmika, (_, _) => $"print(\"{_message}\")");
        _commands.Add(Manufacturers.Doosan, (_, _) => $"tp_log(\"{_message}\")");
    }

    public override string ToString() => $"Command (Message \"{_message}\")";
}
