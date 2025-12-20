package frc.robot.utilities;

import edu.wpi.first.wpilibj.Timer;
import java.net.http.WebSocket;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.CompletableFuture;
import java.util.concurrent.CompletionStage;
import org.littletonrobotics.junction.Logger;

public class WebsocketListener implements WebSocket.Listener {
    List<CharSequence> parts = new ArrayList<>();
    private List<CharSequence> outParts = new ArrayList<>();
    CompletableFuture<?> accumulatedMessage = new CompletableFuture<>();
    private boolean receivingMessages;
    private Timer t;

    public WebsocketListener() {
        t = new Timer();
    }

    public CompletionStage<?> onText(WebSocket webSocket, CharSequence message, boolean last) {
        receivingMessages = true;
        t.restart();

        parts.add(message);
        webSocket.request(1);
        if (last) {
            processWholeText(parts);
            parts = new ArrayList<>();
            accumulatedMessage.complete(null);
            CompletionStage<?> cf = accumulatedMessage;
            accumulatedMessage = new CompletableFuture<>();
            return cf;
        }
        return accumulatedMessage;
    }

    private void processWholeText(List<CharSequence> parts) {
        outParts = parts;
    }

    public String getOutput() {
        Logger.recordOutput("timey", t.get());
        receivingMessages = t.get() < 1;
        return outParts.stream().map((c) -> c.toString()).reduce("", String::concat).toString();
    }

    public boolean isReceivingMessages() {
        return receivingMessages;
    }
}
