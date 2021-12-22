import React, { createContext, useContext, useState, useEffect } from 'react';
import { Participant, Room } from 'twilio-video';

// Add preview participant
type selectedParticipantContextType = [
  Participant | null,
  (participant: Participant) => void,
  string | null,
  (participantName: string) => void
];

export const selectedParticipantContext = createContext<selectedParticipantContextType>(null!);

export default function useSelectedParticipant() {
  const [selectedParticipant, setSelectedParticipant, previewParticipant, setPreviewParticpant] = useContext(
    selectedParticipantContext
  );
  return [selectedParticipant, setSelectedParticipant, previewParticipant, setPreviewParticpant] as const;
}

type SelectedParticipantProviderProps = {
  room: Room | null;
  children: React.ReactNode;
};

export function SelectedParticipantProvider({ room, children }: SelectedParticipantProviderProps) {
  const [selectedParticipant, _setSelectedParticipant] = useState<Participant | null>(null);
  const setSelectedParticipant = (participant: Participant) =>
    _setSelectedParticipant(prevParticipant => (prevParticipant === participant ? null : participant));

  const [previewParticipant, setPreviewParticipant] = useState<string | null>(null);
  //const setPreviewParticipant = (participant: Participant) =>
  //  _setSelectedParticipant(prevParticipant => (prevParticipant === participant ? null : participant));

  useEffect(() => {
    if (room) {
      const onDisconnect = () => {
        _setSelectedParticipant(null);
        setPreviewParticipant(null);
      };
      const handleParticipantDisconnected = (participant: Participant) => {
        _setSelectedParticipant(prevParticipant => (prevParticipant === participant ? null : prevParticipant));
        setPreviewParticipant(prevParticipantName =>
          prevParticipantName === participant.identity ? null : prevParticipantName
        );
      };

      room.on('disconnected', onDisconnect);
      room.on('participantDisconnected', handleParticipantDisconnected);
      return () => {
        room.off('disconnected', onDisconnect);
        room.off('participantDisconnected', handleParticipantDisconnected);
      };
    }
  }, [room]);

  return (
    <selectedParticipantContext.Provider
      value={[selectedParticipant, setSelectedParticipant, previewParticipant, setPreviewParticipant]}
    >
      {children}
    </selectedParticipantContext.Provider>
  );
}
