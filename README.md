# Modna Lagring

- Modenhet av avokadoer vurderes automatisk ved scanning med multispektral
  kamera
- Avokadoer som er spisemodne fordeles mellom fast modne og myke
  modne og gjøres tilgjengelig for kunder i ulike soner oppå maskinen.
- Andre avokadoer fordeles i ulike temperatursoner for modning basert
  på modenhetsgrad og forventet kjøp fra kunder av fast og myk moden avokado
  fremover. Dette er imidlertid en fremtidig plan, og vi trenger ikke ta
  for mye hensyn til dette.
- I denne modulen trenger vi ikke bry oss om alle detaljer ved skanningen,
  dette håndteres andre steder.

Under er mer detaljer for forslag til implementasjon. Ingenting av dette
er strenge krav som må følges.
- Maskinens mål kan f.eks. være ca 70 cm bred og lang, og 80 cm høy.
  - Dette er veldig omtrentelig og kan justeres ettersom hensiktsmessig.
  - Bakenden av maskinen kan være høyere, men frukten kan ikke ligge høyere enn at kundene kan rekke den.
- Jeg ønsker tre ulike seksjoner på toppen med avokadoer: Myke modne,
  faste modne og modne om noen dager.
- Jeg ønsker ca. 3-8 hyller inni maskinen for lagring av avokado.
  Hver hylle burde kunne romme ca 20-50 avokadoer.
- Jeg ønsker å bruke en lineær aktuator som heis, med X og Z-akse
  for også å kunne styre plasseringen på hyllen. Heisen flytter
  avokadoen mellom hyllene, inkludert hyllene på toppen som er
  tilgjengelig for kundene.
- Heisen er på maskinens bakside.
- Avokadoene kan settes inn og tas ut med en gaffel som går i Y-retning.
- Heisen trenger ikke romme mer enn én avokado, og skal ikke
  trenge noen gripemekanisme.
- Jeg ønsker at hver hylle skal være delt inn i rader. Man kan
  f.eks. ha avokado av lik modenhetsgrad på hver rad. Det er
  akseptabelt å ha en LIFO-løsning siden man kan tenke seg
  at man fyller på og tømmer hele rader nært i tid.
- Hyllene må være matsikre og bør være enkle å rengjøre
- De ansatte i butikken fyller på heisen forfra.
- Maskinen skal helst kunne generaliseres til annen frukt, som f.eks.:
  mango, tomat, kiwi og pærer.
- Jeg ønsker at systemet skal være relativt billig og enkelt, gjerne så
  en prototype kan gjennomføres som et DIY prosjekt; samtidig som det kan
  skalere til et større prosjekt senere.
